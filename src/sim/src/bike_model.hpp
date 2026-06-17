#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <iostream>

struct BikeState {
    double x_ddot;
    double y_ddot;
    double x_dot;
    double y_dot;
    double x;
    double y;
    double psi_dot;
    double psi;
    double a_x;
    double front_tire_force;
    double rear_tire_force;
};

struct BikeInputs {
    double delta;
    double psi_dot;
    double throttle;
};

struct ModelParams {
    double mass;
    double lf;
    double lr;
    double Iz;
    double mu;
    double g;
};

class BikeModel
{
public:
    BikeModel(double dt)
        : m_{200.0, 0.781, 0.736, 260.0, 1.0, 9.81}, // TODO: need parameters for our car
          s_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          i_{0.0, 0.0, 0.0},
          dt_{dt},
          last_track_curvature_{0.0},
          last_feasible_{true}
    {
    }

    void set_throttle(double throttle)
    {
        i_.throttle = throttle;
    }

    void update_state(double track_curvature)
    {
        last_track_curvature_ = track_curvature;

        bool is_feasible = evaluate_turning_possibility(track_curvature);
        last_feasible_ = is_feasible;

        if (is_feasible) {
            step_time_forward(track_curvature);
        } else {
            step_time_forward_infeasible();
            RCLCPP_WARN(rclcpp::get_logger("sim_node"),
                        "Turn is not feasible, holding state.");
        }
    }

    double get_x_dot() const
    {
        return s_.x_dot;
    }

    double get_s() const
    {
        return s_.x;
    }

    double get_performance_fraction() const
    {
        double v = get_speed();
        double kappa = std::abs(last_track_curvature_);

        if (kappa < 1e-9) {
            return 0.0;
        }

        double required_lateral_acc = v * v * kappa;
        double max_lateral_acc = m_.mu * m_.g;

        return required_lateral_acc / max_lateral_acc;
    }

    bool was_last_step_feasible() const
    {
        return last_feasible_;
    }

    double get_max_speed_for_curvature(double track_curvature) const
    {
        double kappa = std::abs(track_curvature);

        if (kappa < 1e-9) {
            return std::numeric_limits<double>::infinity();
        }

        return std::sqrt(m_.mu * m_.g / kappa);
    }

private:
    ModelParams m_;
    BikeState s_;
    BikeInputs i_;
    double dt_;

    double last_track_curvature_;
    bool last_feasible_;

    double get_speed() const
    {
        return std::hypot(s_.x_dot, s_.y_dot);
    }

    double get_req_yaw_rate(double track_curvature) const
    {
        double v = get_speed();
        return v * track_curvature;
    }

    double get_req_lateral_acc(double track_curvature) const
    {
        double v = get_speed();
        return v * v * track_curvature;
    }

    double get_req_lateral_force(double track_curvature) const
    {
        return m_.mass * get_req_lateral_acc(track_curvature);
    }

    bool evaluate_turning_possibility(double track_curvature)
    {
        double v = get_speed();
        double kappa = std::abs(track_curvature);

        if (kappa < 1e-9) {
            i_.delta = 0.0;
            s_.psi_dot = 0.0;
            s_.front_tire_force = 0.0;
            s_.rear_tire_force = 0.0;
            return true;
        }

        double required_lateral_acc = v * v * kappa;
        double max_lateral_acc = m_.mu * m_.g;

        if (required_lateral_acc > max_lateral_acc) {
            return false;
        }

        // If feasible, assign the yaw rate needed to follow the track.
        s_.psi_dot = get_req_yaw_rate(track_curvature);

        // Estimate force distribution between front and rear axles.

        double F_total = get_req_lateral_force(track_curvature);
        double L = m_.lf + m_.lr;

        s_.front_tire_force = (m_.lr / L) * F_total;
        s_.rear_tire_force  = (m_.lf / L) * F_total;

        // Approx. steering angle
        i_.delta = std::atan(L * track_curvature);

        return true;
    }

    double eval_x_ddot() const
    {
        // Body-frame longitudinal acceleration.
        return s_.psi_dot * s_.y_dot + s_.a_x;
    }

    double eval_y_ddot() const
    {
        // Simple approach

        double F_y_total = s_.front_tire_force * std::cos(i_.delta)
                         + s_.rear_tire_force;

        return -s_.psi_dot * s_.x_dot + F_y_total / m_.mass;
    }

    void step_time_forward(double track_curvature)
    {
        s_.a_x = i_.throttle;

        s_.psi_dot = get_req_yaw_rate(track_curvature);

        double x_ddot = eval_x_ddot();
        double y_ddot = eval_y_ddot();

        s_.x_ddot = x_ddot;
        s_.y_ddot = y_ddot;

        s_.x_dot += s_.x_ddot * dt_;
        s_.y_dot += s_.y_ddot * dt_;

        // Prevent tiny numerical backward velocity if throttle/braking drives below zero.
        if (s_.x_dot < 0.0) {
            s_.x_dot = 0.0;
        }

        s_.x += s_.x_dot * dt_;
        s_.y += s_.y_dot * dt_;
        s_.psi += s_.psi_dot * dt_;

        std::cout << "x_dot: " << s_.x_dot
                  << ", y_dot: " << s_.y_dot
                  << ", psi_dot: " << s_.psi_dot
                  << ", performance: " << get_performance_fraction()
                  << std::endl;
        std::cout << "x: " << s_.x
                  << ", y: " << s_.y
                  << ", psi: " << s_.psi
                  << std::endl;
    }

    void step_time_forward_infeasible()
    {
        // Brake

        constexpr double braking_acc = -0.5; // [m/s^2]

        s_.a_x = braking_acc;
        s_.x_ddot = braking_acc;
        s_.y_ddot = 0.0;

        s_.x_dot += s_.x_ddot * dt_;

        if (s_.x_dot < 0.0) {
            s_.x_dot = 0.0;
        }

        // Do not increase yaw rate further if the turn is infeasible.
        s_.x += s_.x_dot * dt_;
        s_.psi += s_.psi_dot * dt_;
    }
};
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
    double mu_long;
    double mu_lat;
    double g;

    double wheel_radius;
    double max_drive_torque;
    double max_brake_torque;
    double Crr;
    double rho_air;
    double CdA;
};

class BikeModel
{
public:
    BikeModel(double dt)
        : m_{.mass = 200.0,
            .lf = 0.781,
            .lr = 0.736,
            .Iz = 260.0,
            .mu_long = 0.8,
            .mu_lat = 1.63,
            .g = 9.81,
            .wheel_radius = 0.19,
            .max_drive_torque = 150.0,
            .max_brake_torque = 200.0,
            .Crr = 0.015,
            .rho_air = 1.225,
            .CdA = 1.2},
          s_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          i_{0.0, 0.0, 0.0},
          dt_{dt},
          last_track_curvature_{0.0},
          last_feasible_{true}
    {
    }

    void set_throttle(double throttle)
    {
        i_.throttle = std::clamp(throttle, -1.0, 1.0);
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

        double a_y_req = v * v * kappa;
        double a_x_req = s_.a_x;

        double a_y_max = m_.mu_lat * m_.g;
        double a_x_max = m_.mu_long * m_.g;

        if (a_y_max < 1e-9 || a_x_max < 1e-9) {
            return std::numeric_limits<double>::infinity();
        }

        return std::sqrt(
            (a_x_req * a_x_req) / (a_x_max * a_x_max) +
            (a_y_req * a_y_req) / (a_y_max * a_y_max)
        );
    }

    bool was_last_step_feasible() const
    {
        return last_feasible_;
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

        double a_y_req = v * v * kappa;

        // Estimate longitudinal acceleration from current throttle/brake command.
        double a_x_req = eval_longitudinal_acc();

        double a_y_max = m_.mu_lat * m_.g;
        double a_x_max = m_.mu_long * m_.g;

        double usage = std::sqrt(
            (a_x_req * a_x_req) / (a_x_max * a_x_max) +
            (a_y_req * a_y_req) / (a_y_max * a_y_max)
        );

        if (usage > 1.0) {
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
    
    double eval_longitudinal_acc() const
    {
        double torque_cmd = std::clamp(i_.throttle, -1.0, 1.0);
        double vx = s_.x_dot;

        // Avoid resistance causing the vehicle to roll backward from rest.
        if (vx <= 1e-6 && torque_cmd <= 0.0) {
            return 0.0;
        }

        double F_drive = 0.0;

        if (torque_cmd >= 0.0) {
            // Engine/motor supplies requested torque up to max_drive_torque.
            double wheel_torque = torque_cmd * m_.max_drive_torque;

            F_drive = wheel_torque / m_.wheel_radius;
        } else {
            // Negative command gives braking torque.
            double brake_torque = torque_cmd * m_.max_brake_torque;
            F_drive = brake_torque / m_.wheel_radius;
        }

        // Rolling resistance opposes motion.
        double F_roll = m_.Crr * m_.mass * m_.g;

        // Aerodynamic drag opposes motion.
        double F_drag = 0.5 * m_.rho_air * m_.CdA * vx * vx;

        double F_resist = F_roll + F_drag;

        // Resistive forces oppose positive forward motion.
        double F_long = F_drive - F_resist;

        // Do not let resistance alone create backward acceleration at rest.
        if (vx <= 1e-6 && F_long < 0.0) {
            F_long = 0.0;
        }

        return F_long / m_.mass;
    }

    void step_time_forward(double track_curvature)
    {
        s_.a_x = eval_longitudinal_acc();
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include "track_srv/srv/return_track.hpp"

#include "spline_library/splines/uniform_cr_spline.h"
#include "spline_library/utils/arclength.h"
#include "spline_library/utils/splineinverter.h"

#include "lfs_msgs/msg/bike_state.hpp"
#include "bike_model.hpp"

class SimNode : public rclcpp::Node
{
public:
    SimNode() : Node("sim_node"), bike_model_(dt_)
    {
        bike_model_ = BikeModel(dt_);

        create_client_and_get_track_points();

        if (!create_client_and_get_track_points()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to retrieve track points, aborting node construction");
            throw std::runtime_error("Failed to retrieve track points");
        }
        create_spline_from_points();

        // Initialize state publisher
        state_publisher_ = this->create_publisher<lfs_msgs::msg::BikeState>("current_state", 10);
        vis_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualize_pose", 10);

        // Initialize subscriber to throttle commands
        throttle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("throttle_cmd", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                // Handle throttle command
                bike_model_.set_throttle(msg->data);
            });

        // Initialize timer to update state at fixed intervals
        state_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            [this]() {
                update_state();
            });

    }

    void update_state()
    {
        // Update the bike model state based on the current throttle and track

        bike_model_.update_state(track_curvature_at_current_position());

        // Publish the current state
        auto state_msg = lfs_msgs::msg::BikeState();
        state_msg.x_dot = bike_model_.get_x_dot();
        state_msg.s = bike_model_.get_s();
        state_msg.performance_fraction = bike_model_.get_performance_fraction();
        state_publisher_->publish(state_msg);

		visualization_msgs::msg::Marker pose;
		pose.type = 0;
		pose.ns = "test";
		pose.id = 1;
		pose.action = 0;
		pose.scale.x = 3;
		pose.scale.y = 1;
		pose.scale.z = 1;
		pose.lifetime = rclcpp::Duration(1, 0);
		// geometry_msgs::msg::PoseStamped pose;
		static double time = 0;
		time += dt_;
		pose.header.stamp = rclcpp::Time(time);
		pose.header.frame_id = "map";
        double s = bike_model_.get_s();
        auto itc = spline_->getCurvature(ArcLength::solveLengthCyclic(*spline_, 0.0f, static_cast<float>(s)));
		pose.pose.position.x = itc.position.x();
		pose.pose.position.y = itc.position.y();
		pose.pose.position.z = 0;
		double yaw = std::atan2(itc.tangent.y(), itc.tangent.x());
		Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
		pose.pose.orientation.x = q.x();
		pose.pose.orientation.y = q.y();
		pose.pose.orientation.z = q.z();
		pose.pose.orientation.w = q.w();


		if (bike_model_.get_performance_fraction() > 1){
			pose.color.g = 0;
			pose.color.r = 1;
		}
		else{
			pose.color.g = bike_model_.get_performance_fraction();
			pose.color.r = 0;
			pose.color.b = 0.3;
		}
		pose.color.a = 1;
        vis_publisher_->publish(pose);
    }

    double track_curvature_at_current_position()
    {
        // Get the current position along the track
        double s = bike_model_.get_s();

        // Use the spline to get the curvature at the current position
        auto itc = spline_->getCurvature(ArcLength::solveLengthCyclic(*spline_, 0.0f, static_cast<float>(s)));
        Eigen::Vector2d pos = itc.position;
        Eigen::Vector2d d1  = itc.tangent;         // first derivative (not unit)
        Eigen::Vector2d d2  = itc.curvature;       // second derivative vector

        double denom = std::pow(d1.norm(), 3);
        double kappa = (denom > 1e-12) ? (d1(0)*d2(1) - d1(1)*d2(0)) / denom : 0.0;

        if (std::abs(denom) < 1e-6) {
            return 0.0; // Avoid division by zero, treat as straight line
        }

        return kappa;
    }

    bool create_client_and_get_track_points()
    {
        // Get spline points from track_srv
        // create client for the return_track service
        client_ = this->create_client<track_srv::srv::ReturnTrack>("return_track");

        // wait for the service to appear
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service 'return_track' not available");
            return false;
        }

        auto request = std::make_shared<track_srv::srv::ReturnTrack::Request>();
        // send request asynchronously with a callback to receive the points
        auto result_future = client_->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::milliseconds(5));
        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call return_track service");
            return false;
        }

        auto response = result_future.get();
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call return_track service (empty response)");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu points from return_track", response->points.size());

        if (response->points.size() < 4) {
            RCLCPP_ERROR(this->get_logger(), "Received too few points (%zu). Need at least 4 to build spline.", response->points.size());
            return false;
        }

        // store received points
        Eigen::Vector2d point;
        for (const auto & p : response->points) {
            point.x() = p.x;
            point.y() = p.y;
            track_points_.push_back(point);
        }

        return true;
    }

    void create_spline_from_points()
    {
        // Create a spline from the retrieved track points
        // Use the spline_library to create a uniform cubic spline
        spline_ = std::make_unique<LoopingUniformCRSpline<Eigen::Vector2d>>(track_points_);
    }

private:
    // Declare member variables for publishers, subscribers, and timers
    rclcpp::Client<track_srv::srv::ReturnTrack>::SharedPtr client_;
    std::vector<Eigen::Vector2d> track_points_;
    std::unique_ptr<LoopingUniformCRSpline<Eigen::Vector2d>> spline_;

    rclcpp::Publisher<lfs_msgs::msg::BikeState>::SharedPtr state_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_subscriber_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    BikeModel bike_model_;
    double dt_ = 0.005; // Set the time step for the simulation
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

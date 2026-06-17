#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("longitudinal_controller")
    {
        // parameters: topic name, publish rate (Hz), open-loop throttle value
        this->declare_parameter<std::string>("throttle_topic", "/throttle_cmd");
        this->declare_parameter<double>("publish_hz", 50.0);
        this->declare_parameter<double>("throttle_value", 0.3);

        throttle_topic_ = this->get_parameter("throttle_topic").as_string();
        publish_hz_ = this->get_parameter("publish_hz").as_double();
        throttle_value_ = this->get_parameter("throttle_value").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Float64>(throttle_topic_, 10);

        auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&ControllerNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Controller node started. Publishing on '%s' at %.1f Hz (throttle=%.3f)",
                    throttle_topic_.c_str(), publish_hz_, throttle_value_);
    }

private:
    void timerCallback()
    {
        auto msg = std_msgs::msg::Float64();
        // open-loop throttle command; simulator will read this topic
        msg.data = throttle_value_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string throttle_topic_;
    double publish_hz_;
    double throttle_value_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
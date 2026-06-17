#include "rclcpp/rclcpp.hpp"
#include "track_srv/srv/return_track.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;


static std::vector<geometry_msgs::msg::Point> test;


void return_track(
    const std::string & track,
    const std::shared_ptr<track_srv::srv::ReturnTrack::Request> request,
    std::shared_ptr<track_srv::srv::ReturnTrack::Response> response)
{
    (void) request;

    // Expect track files to be located in a "tracks" directory relative to
    // the current working directory. Example filename: tracks/FSG2023.track
    const std::string filename = std::string("tracks/") + track + ".track";
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("return_track_server"),
            "Could not open track file: %s",
            filename.c_str());
        return;
    }

    std::string line;
    std::size_t line_no = 0;
	test.clear();
    while (std::getline(ifs, line)) {
        ++line_no;
        // trim leading whitespace
        auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            continue; // empty line
        }
        if (line[first] == '#') {
            continue; // comment line
        }

        std::istringstream ss(line);
        std::string label;
        double x, y, z = 0.0;
        if (!(ss >> label >> x >> y)) {
            RCLCPP_WARN(
                rclcpp::get_logger("return_track_server"),
                "Skipping malformed line %zu in %s",
                line_no, filename.c_str());
            continue;
        }
        // optional z
        if (!(ss >> z)) {
            z = 0.0;
        }
        
        RCLCPP_INFO(
            rclcpp::get_logger("return_track_server"),
            "Read point: %f, %f, %s",
            x, y, label.c_str());

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
		test.push_back(point);
        response->points.push_back(point);
    }

    RCLCPP_INFO(
        rclcpp::get_logger("return_track_server"),
        "Read %zu points from %s",
        response->points.size(),
        filename.c_str());
}
void visualize(rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub){
	geometry_msgs::msg::PolygonStamped pol;

	for (auto &p : test){
		geometry_msgs::msg::Point32 p32;
		p32.x = static_cast<float>(p.x);
		p32.y = static_cast<float>(p.y);
		p32.z = static_cast<float>(p.z);
		pol.polygon.points.push_back(p32);
	}

	// pol.polygon.points = test;
	pol.header.frame_id = "map";
	pub->publish(pol);
}


int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("return_track_server");

    // declare a parameter named "track" so the node can accept it when launched:
    // Example usage:
    //   ros2 run track_srv server --ros-args -p track:=my_track_name
    node->declare_parameter<std::string>("track", std::string());
    std::string track;
    node->get_parameter("track", track);

    if (track.empty()) {
        track.assign("FSG2023");
    }

    RCLCPP_INFO(
        rclcpp::get_logger("return_track_server"),
        "Using track: %s",
        track.c_str());

    auto service = node->create_service<track_srv::srv::ReturnTrack>(
        "return_track",
        [track](
            const std::shared_ptr<track_srv::srv::ReturnTrack::Request> request,
            std::shared_ptr<track_srv::srv::ReturnTrack::Response> response)
        {
            return_track(track, request, response);
        });

	auto pub = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
			"/visualize_track", 10);
	auto timer = node->create_wall_timer(1000ms, 
			[pub](){
			visualize(pub);
			});

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

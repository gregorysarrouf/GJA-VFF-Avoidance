#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

class AvoidanceNode : public rclcpp::Node {
public:
    AvoidanceNode();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();
    geometry_msgs::msg::Point calculate_attractive_force(
	const geometry_msgs::msg::Point& robot_position,
	const geometry_msgs::msg::Point& goal);
    geometry_msgs::msg::Point calculate_repulsive_force(
	const geometry_msgs::msg::Point& robot_position,
	const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publish_debug_markers(
        const geometry_msgs::msg::Point& robot_position,
        const geometry_msgs::msg::Point& attractive_force,
        const geometry_msgs::msg::Point& repulsive_force,
        const geometry_msgs::msg::Point& resultant_force);
    visualization_msgs::msg::Marker create_force_marker(
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& force,
        int id,
        float r, float g, float b);

    // Parameters
    const double K_att_ = 1.5;  // Attractive force gain
    const double K_rep_ = 0.5;  // Repulsive force gain
    const double d_0_ = 10.0;    // Obstacle influence distance
    const double max_linear_speed_ = 0.3 * 0.5;  // Maximum linear speed (m/s)
    const double max_angular_speed_ = 2.5 * 0.5; // Maximum angular speed (rad/s)

    // Goal position
    geometry_msgs::msg::Point goal_;

    // ROS 2 publisher, subscriber, and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest laser scan data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    rclcpp::Time latest_scan_time_;
};

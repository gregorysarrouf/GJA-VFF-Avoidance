#include "../../include/vff_avoidance/AvoidanceNode.hpp"

AvoidanceNode::AvoidanceNode() : Node("vff_obstacle_avoidance") {
    // Create a publisher for the /cmd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a subscriber for the /scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AvoidanceNode::scan_callback, this, _1));

    // Create a publisher for debugging markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug_markers", 10);

    // Initialize goal position
    goal_.x = 2.0;
    goal_.y = 0.0;

    // Create a timer to execute the algorithm at 20Hz
    timer_ = this->create_wall_timer(50ms, std::bind(&AvoidanceNode::timer_callback, this));
} 

void AvoidanceNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the latest laser scan message
    latest_scan_ = msg;
    latest_scan_time_ = this->now();
}

void AvoidanceNode::timer_callback() {	
    // Check if the laser scan message is valid
    if (!latest_scan_ || (this->now() - latest_scan_time_).seconds() > 2.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid or outdated laser scan data.");
        return;
    }

    // Robot's current position (assumed to be at the origin for simplicity)
    geometry_msgs::msg::Point robot_position;
    robot_position.x = 0.0;
    robot_position.y = 0.0;

    // Calculate attractive force
    geometry_msgs::msg::Point attractive_force = calculate_attractive_force(robot_position, goal_);

    // Calculate repulsive force
    geometry_msgs::msg::Point repulsive_force = calculate_repulsive_force(robot_position, latest_scan_);

    // Calculate resultant force
    geometry_msgs::msg::Point resultant_force;
    resultant_force.x = attractive_force.x + repulsive_force.x;
    resultant_force.y = attractive_force.y + repulsive_force.y;

    // Normalize the resultant force
    double force_magnitude = std::hypot(resultant_force.x, resultant_force.y);
    if (force_magnitude > 0) {
        resultant_force.x /= force_magnitude;
        resultant_force.y /= force_magnitude;
    }

    // Use the resultant force to compute linear and angular speeds
    double linear_speed = resultant_force.x * max_linear_speed_;
    double angular_speed = resultant_force.y * max_angular_speed_;

    // Clamp speeds to a safe range
    linear_speed = std::clamp(linear_speed, -max_linear_speed_, max_linear_speed_);
    angular_speed = std::clamp(angular_speed, -max_angular_speed_, max_angular_speed_);

    // Create a Twist message to control the robot
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;

    // Publish the Twist message
    publisher_->publish(twist);

    // Publish debugging markers if there are subscribers
    if (marker_publisher_->get_subscription_count() > 0) {
        publish_debug_markers(robot_position, attractive_force, repulsive_force, resultant_force);
    }   
}

geometry_msgs::msg::Point AvoidanceNode::calculate_attractive_force(
    const geometry_msgs::msg::Point& robot_position,
    const geometry_msgs::msg::Point& goal) {
        geometry_msgs::msg::Point force;
        force.x = K_att_ * (goal.x - robot_position.x);
        force.y = K_att_ * (goal.y - robot_position.y);
        return force;
}

geometry_msgs::msg::Point AvoidanceNode::calculate_repulsive_force(
    const geometry_msgs::msg::Point& robot_position,
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        geometry_msgs::msg::Point force;
        force.x = 0.0;
        force.y = 0.0;

	// Limit field of view 
        double vision_cone_width = M_PI;
        double half_width = vision_cone_width / 2.0;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isinf(scan->ranges[i])) {
                continue;  // Ignore infinite values
            }

            double angle = scan->angle_min + i * scan->angle_increment;

            if (angle >= -half_width && angle <= half_width) {
                double obstacle_distance = scan->ranges[i];

                if (obstacle_distance < d_0_) {
                    // Calculate repulsive force magnitude
                    double repulsive_magnitude = K_rep_ * (1.0 / obstacle_distance - 1.0 / d_0_) * (1.0 / (obstacle_distance * obstacle_distance));
		   
                    // Smooth the repulsive force
                    double smoothing_factor = 0.05;
                    force.x -= smoothing_factor * repulsive_magnitude * std::cos(angle);
                    force.y -= smoothing_factor * repulsive_magnitude * std::sin(angle);
                }
            }
        }

        return force;
}

void AvoidanceNode::publish_debug_markers(
    const geometry_msgs::msg::Point& robot_position,
    const geometry_msgs::msg::Point& attractive_force,
    const geometry_msgs::msg::Point& repulsive_force,
    const geometry_msgs::msg::Point& resultant_force) {
        // Create a MarkerArray message
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // Add attractive force marker
        auto attractive_marker = create_force_marker(robot_position, attractive_force, 0, 0.0, 1.0, 0.0); // Green
        marker_array.markers.push_back(attractive_marker);

        // Add repulsive force marker
        auto repulsive_marker = create_force_marker(robot_position, repulsive_force, 1, 1.0, 0.0, 0.0); // Red
        marker_array.markers.push_back(repulsive_marker);

        // Add resultant force marker
        auto resultant_marker = create_force_marker(robot_position, resultant_force, 2, 0.0, 0.0, 1.0); // Blue
        marker_array.markers.push_back(resultant_marker);


        // Publish the MarkerArray
        marker_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker AvoidanceNode::create_force_marker(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& force,
    int id,
    float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // Use the robot's frame
        marker.header.stamp = this->now();
        marker.ns = "forces";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the start and end points of the arrow
        geometry_msgs::msg::Point end;
        end.x = start.x + force.x;
        end.y = start.y + force.y;
        end.z = 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);

        
        marker.scale.x = 0.05;
        marker.scale.y = 0.07;
        marker.scale.z = 0.15;

        marker.color.a = 1.0;  // Alpha (opacity)
        marker.color.r = r;    // Red
        marker.color.g = g;    // Green
        marker.color.b = b;    // Blue

        return marker;
}

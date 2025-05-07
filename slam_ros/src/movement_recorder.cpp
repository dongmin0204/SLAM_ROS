#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <string>

class MovementRecorder : public rclcpp::Node
{
public:
    MovementRecorder() : Node("movement_recorder")
    {
        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&MovementRecorder::odom_callback, this, std::placeholders::_1));

        // Initialize file for recording
        movement_file_.open("robot_movements.csv");
        movement_file_ << "timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
        
        RCLCPP_INFO(this->get_logger(), "Movement recorder node started");
    }

    ~MovementRecorder()
    {
        if (movement_file_.is_open())
        {
            movement_file_.close();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Record position and orientation
        movement_file_ << msg->header.stamp.sec << "."
                      << msg->header.stamp.nanosec << ","
                      << msg->pose.pose.position.x << ","
                      << msg->pose.pose.position.y << ","
                      << msg->pose.pose.position.z << ","
                      << msg->pose.pose.orientation.x << ","
                      << msg->pose.pose.orientation.y << ","
                      << msg->pose.pose.orientation.z << ","
                      << msg->pose.pose.orientation.w << "\n";
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream movement_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovementRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
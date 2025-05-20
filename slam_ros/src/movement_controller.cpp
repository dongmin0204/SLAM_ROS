#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <slam_ros/action/move_xy.hpp>
#include <fstream>
#include <string>
#include <cmath>

using MoveXY = slam_ros::action::MoveXY;
using GoalHandleMoveXY = rclcpp_action::ServerGoalHandle<MoveXY>;

class MovementController : public rclcpp::Node
{
public:
    MovementController() : Node("movement_controller")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<MoveXY>(
            this,
            "move_xy",
            std::bind(&MovementController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MovementController::handle_cancel, this, std::placeholders::_1),
            std::bind(&MovementController::handle_accepted, this, std::placeholders::_1));

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&MovementController::odom_callback, this, std::placeholders::_1));

        // Initialize file for recording
        movement_file_.open("robot_movements.csv");
        movement_file_ << "timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
        
        RCLCPP_INFO(this->get_logger(), "Movement controller node started");
    }

    ~MovementController()
    {
        if (movement_file_.is_open())
        {
            movement_file_.close();
        }
    }

private:
    rclcpp_action::Server<MoveXY>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream movement_file_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveXY::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: x=%.2f, y=%.2f", goal->x, goal->y);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveXY> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveXY> goal_handle)
    {
        std::thread{std::bind(&MovementController::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveXY> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveXY::Feedback>();
        auto result = std::make_shared<MoveXY::Result>();

        // Simulate movement (replace with actual robot control)
        double step = 0.05;  // 5 cm per iteration
        double distance = std::hypot(goal->x, goal->y);
        int iterations = static_cast<int>(std::ceil(distance / step));
        double rate_hz = 20.0;
        double dt = 1.0 / rate_hz;

        for (int i = 0; i < iterations && rclcpp::ok(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            double progress = static_cast<double>(i + 1) / iterations;
            feedback->current_x = goal->x * progress;
            feedback->current_y = goal->y * progress;
            goal_handle->publish_feedback(feedback);

            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }

        if (rclcpp::ok())
        {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovementController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
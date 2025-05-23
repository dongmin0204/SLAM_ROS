#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <slam_ros/action/move_xy.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "quintic_trajectory.hpp"
#include "kinematic_model.hpp"
#include "mpc_solver.hpp"
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

        // Create publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);

        // Create timer for MPC control
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&MovementController::control_callback, this));

        // Initialize file for recording
        movement_file_.open("robot_movements.csv");
        movement_file_ << "timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
        
        // Initialize MPC solver
        MpcSolver::MpcConfig mpc_config;
        mpc_config.prediction_horizon = 10;
        mpc_config.dt = 0.05;
        mpc_config.Q = Eigen::Matrix3d::Identity();
        mpc_config.Rd = Eigen::Matrix3d::Identity() * 0.1;
        mpc_config.v_max = 0.9;
        mpc_config.omega_max = 22.99;
        mpc_solver_ = std::make_unique<MpcSolver>(mpc_config);
        
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream movement_file_;
    std::unique_ptr<MpcSolver> mpc_solver_;
    std::unique_ptr<QuinticTrajectory> trajectory_generator_;
    
    // Current state
    Eigen::Vector3d current_state_;  // [x, y, theta]
    std::vector<QuinticTrajectory::State> reference_trajectory_;
    bool is_controlling_ = false;
    std::shared_ptr<GoalHandleMoveXY> current_goal_handle_;

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
        is_controlling_ = false;
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
        current_goal_handle_ = goal_handle;
        
        // Generate trajectory
        QuinticTrajectory::State start_state;
        start_state.x = current_state_[0];
        start_state.y = current_state_[1];
        start_state.theta = current_state_[2];
        start_state.vx = 0.0;
        start_state.vy = 0.0;
        start_state.omega = 0.0;

        QuinticTrajectory::State goal_state;
        goal_state.x = goal->x;
        goal_state.y = goal->y;
        goal_state.theta = std::atan2(goal->y - start_state.y, goal->x - start_state.x);
        goal_state.vx = 0.0;
        goal_state.vy = 0.0;
        goal_state.omega = 0.0;

        // Generate trajectory
        reference_trajectory_ = trajectory_generator_->generate(start_state, goal_state, 5.0);  // 5 seconds
        is_controlling_ = true;
    }

    void control_callback()
    {
        if (!is_controlling_ || reference_trajectory_.empty())
            return;

        // Get current reference state
        auto ref_state = reference_trajectory_[0];
        Eigen::Vector3d ref_vec(ref_state.x, ref_state.y, ref_state.theta);

        // Compute control input using MPC
        auto control = mpc_solver_->computeControl(current_state_, {ref_vec});

        // Publish control command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = control[0];
        cmd_vel.linear.y = control[1];
        cmd_vel.angular.z = control[2];
        cmd_vel_pub_->publish(cmd_vel);

        // Check if goal is reached
        double pos_error = std::hypot(current_state_[0] - ref_state.x, 
                                    current_state_[1] - ref_state.y);
        double angle_error = std::abs(current_state_[2] - ref_state.theta);
        
        if (pos_error < 0.03 && angle_error < 0.087)  // 3cm and 5 degrees
        {
            is_controlling_ = false;
            auto result = std::make_shared<MoveXY::Result>();
            result->success = true;
            current_goal_handle_->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current state
        current_state_[0] = msg->pose.pose.position.x;
        current_state_[1] = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_state_[2] = yaw;

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
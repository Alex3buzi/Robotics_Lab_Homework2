// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class KDLActionServer : public rclcpp::Node
{
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    explicit KDLActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ros2_kdl_node", options)
    {
        // Declare and retrieve node parameters (command interface and control type)
        this->declare_parameter("cmd_interface", "velocity");
        this->declare_parameter("ctrl", "velocity_ctrl_null"); 
        
        this->get_parameter("cmd_interface", cmd_interface_);
        this->get_parameter("ctrl", ctrl_);

        RCLCPP_INFO(this->get_logger(), "Interface: %s, Control: %s", cmd_interface_.c_str(), ctrl_.c_str());

        // Initialize the robot model from the robot_description parameter
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve robot_description param!");
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        
        // Set joint limits for the iiwa robot
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;
        robot_->setJntLimits(q_min,q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_velocities_cmd_.resize(nj);

        // Subscribe to joint states
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&KDLActionServer::joint_state_callback, this, std::placeholders::_1));

        // Wait for the first joint state message to ensure the robot model is ready
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        // Initialize KDL kinematics and the controller
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        robot_->addEE(KDL::Frame::Identity());
        controller_ = std::make_shared<KDLController>(*robot_);

        // Create the command publisher based on the interface type
        if(cmd_interface_ == "velocity"){
            cmd_pub_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        } else if(cmd_interface_ == "position"){
            cmd_pub_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }
        
        // Initialize the Action Server
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "execute_trajectory",
            std::bind(&KDLActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KDLActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&KDLActionServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "KDL Action Server Ready!");
        // Note: No timer is started here. The robot waits for a goal.
    }

private:
    // --- Member Variables ---
    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmd_pub_;
    
    KDL::JntArray joint_positions_, joint_velocities_, joint_velocities_cmd_;
    bool joint_state_available_ = false;
    std::string cmd_interface_, ctrl_;

    // --- Action Server Callbacks ---

    // Accept the goal request
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: Move to [%.2f, %.2f, %.2f] in %.2f s", 
            goal->end_position_x, goal->end_position_y, goal->end_position_z, goal->traj_duration);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellation requests
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory>)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Start execution in a separate thread to avoid blocking the executor
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        std::thread{std::bind(&KDLActionServer::execute, this, goal_handle)}.detach();
    }

    // --- Execution Logic (The Control Loop) ---
    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // Configure the Planner using the parameters received in the Goal
        Eigen::Vector3d init_position(robot_->getEEFrame().p.data);
        Eigen::Vector3d end_position(goal->end_position_x, goal->end_position_y, goal->end_position_z);
        
        // Note: acc_duration is retrieved from the goal
        double acc_duration = goal->acc_duration; 
        KDLPlanner planner(goal->traj_duration, acc_duration, init_position, end_position);

        double t = 0.0;
        double dt = 0.01; // 100 Hz loop
        rclcpp::Rate loop_rate(100);

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");

        while (rclcpp::ok() && t < goal->traj_duration) 
        {
            // Handle cancellation request
            if (goal_handle->is_canceling()) {
                stop_robot();
                result->success = false;
                result->message = "Canceled";
                goal_handle->canceled(result);
                return;
            }

            // Compute the trajectory point for the current time
            trajectory_point p = planner.linear_traj_trapezoidal(t);
            // Alternative: use cubic if specified

            // Update robot state from joint readings
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute error between current and desired pose
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));

            // Compute control command (Velocity Control)
            if (cmd_interface_ == "velocity") 
            {
                if (ctrl_ == "velocity_ctrl_null") {
                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(p.pos, goal->kp, joint_positions_.data);
                } else {
                    // Fallback to standard velocity control if needed
                    // joint_velocities_cmd_.data = controller_->velocity_ctrl(p.pos, p.vel, goal->kp, ...);
                }

                // Publish command
                FloatArray cmd_msg;
                cmd_msg.data = toStdVector(joint_velocities_cmd_.data);
                cmd_pub_->publish(cmd_msg);
            }

            // Publish feedback to the client
            feedback->position_error = error.norm();
            goal_handle->publish_feedback(feedback);

            t += dt;
            loop_rate.sleep();
        }

        // Trajectory finished: stop the robot and return success
        stop_robot(); 

        result->success = true;
        result->message = "Trajectory Finished";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg->position.size(); i++){
            joint_positions_.data[i] = sensor_msg->position[i];
            joint_velocities_.data[i] = sensor_msg->velocity[i];
        }
    }

    void stop_robot() {
        if(cmd_interface_ == "velocity") {
            FloatArray cmd_msg;
            cmd_msg.data.assign(7, 0.0); // Zero velocity for all 7 joints
            cmd_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KDLActionServer>());
    rclcpp::shutdown();
    return 0;
}

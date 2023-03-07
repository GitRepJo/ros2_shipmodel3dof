// Copyright (c) 2023 Jonas Mahler
	
// This file is part of shipmodel3dof.

// shipmodel3dof is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// shipmodel3dof is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with shipmodel3dof. If not, see <https://www.gnu.org/licenses/>. 

#ifndef SIMULATION_HPP_
#define SIMULATION_HPP_

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"  
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ella_interfaces/msg/full_steering.hpp"

#include <odeint_sim.hpp>
#include <nomoto_ode.hpp>
#include <odeint_obs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @class Simulation
 * @brief Creates a simulation object
 */
class Ros2Simulation : public rclcpp::Node
{
public:
    
    /**
     * @brief A constructor for Simulation class
     * @param options Additional options to control creation of the node.
     */
    explicit Ros2Simulation(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for Simulation class
     */
    ~Ros2Simulation();

    OdeintSim::resultSim resSim;
    OdeintSim simulation;

protected:
 
    /* Parameters used for the ROS2 interface of the simulation*/
    struct paramROS2
    {      
        std::string frame_id       = ""; // Frame id of the pose and path message
        std::string topic_steering = ""; // Input topic for steering message
        std::string topic_pose     = ""; // Output topic for pose message
        std::string topic_path     = ""; // Output topic for path message
        std::string equ_file       = ""; // Input path for equation config file
        std::string sim_file       = ""; // Input path for simulation config file
        int frequency              = 0 ; // Frequency with which to execute the simulation
    };

    /**
     * @brief Read parameters for ROS2 interface out of config file
     * @param config_file string custom config file .yaml matching paramROS2 struct
     * @return paramROS2 struct that describes ROS2 interface 
     */
    paramROS2 read(std::string config_file);

    /**
     * @brief Ros2 main loop to call simulation interface and publish path and pose
     * @param 
     * @return -
     */
    void sim_callback();

       /**
     * @brief Update continously parameters of the equation based on yaml configuration file
     * @param -
     * @return -
     */
    void read_callback();

     /**
     * @brief Update the steering information
     * @param msg custom ros2 message containing velocity and rudder angle
     * @return -
     */
    void steering_callback(const ella_interfaces::msg::FullSteering::SharedPtr msg);

    std::string config_file; // Config file for this node

    std::string frame_id; // Frame id of the published topics

    rclcpp::TimerBase::SharedPtr sim_timer; // Timer to call simulation funtion
    rclcpp::TimerBase::SharedPtr read_timer; // Timer to call read function for configuration files

    rclcpp::Subscription<ella_interfaces::msg::FullSteering>::SharedPtr subscription_steering; // ROS2 steering subscriber
      
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose; // ROS2 pose publisher
    geometry_msgs::msg::PoseStamped pose_message = geometry_msgs::msg::PoseStamped();  

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path; // ROS2 path publisher
    nav_msgs::msg::Path path_message;  

};

#endif //SIMULATION_HPP_
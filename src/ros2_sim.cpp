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

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <ros2_sim.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

Ros2Simulation::Ros2Simulation(const rclcpp::NodeOptions& options) : Node("nomoto",options)
{
    std::string pck_share_dir = ament_index_cpp::get_package_share_directory("shipmodel3dof");

    this->declare_parameter<std::string>("config_file", pck_share_dir + "/ros2/test_nomoto_config.yaml");
    this->get_parameter("config_file", config_file);

    Ros2Simulation::paramROS2 pROS2 = Ros2Simulation::read(config_file);

    sim_timer = this->create_wall_timer(
      std::chrono::milliseconds(pROS2.frequency), std::bind(&Ros2Simulation::sim_callback, this));
    
    read_timer = this->create_wall_timer(
      1000ms, std::bind(&Ros2Simulation::read_callback, this));
    
    frame_id = pROS2.frame_id;
   
    subscription_steering = this->create_subscription<ella_interfaces::msg::FullSteering>(
    pROS2.topic_steering, 10, std::bind(&Ros2Simulation::steering_callback, this, _1));

    publisher_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    pROS2.topic_pose, 10);

    publisher_path = this->create_publisher<nav_msgs::msg::Path>(
    pROS2.topic_path, 10);

    simulation.pNomoto = simulation.nOde.read(pROS2.equ_file);
    simulation.nOde.set(simulation.pNomoto);
    simulation.pSim = simulation.readSimulation(pROS2.sim_file);
}

Ros2Simulation::~Ros2Simulation(){}
  
Ros2Simulation::paramROS2 Ros2Simulation::read(std::string config_file)
{
    Ros2Simulation::paramROS2 pROS2; // Parameter for ROS2

    YAML::Node config = YAML::LoadFile(config_file);

    pROS2.frame_id = config["frame_id"].as<std::string>();
    
    pROS2.topic_steering  = config["topic_listen_steering"].as<std::string>();
    pROS2.topic_pose  = config["topic_publish_pose"].as<std::string>();
    pROS2.topic_path  = config["topic_publish_path"].as<std::string>();
    
    pROS2.equ_file   = config["equ_file"].as<std::string>();
    pROS2.sim_file = config["sim_file"].as<std::string>();
    pROS2.frequency = config["frequency"].as<int>();

    return pROS2;
}
void Ros2Simulation::steering_callback(const ella_interfaces::msg::FullSteering::SharedPtr msg)
{
    simulation.pNomoto.delta = msg->rudder_port; // Get new rudder angle taken from message, use only one rudder
    simulation.pNomoto.rpm = msg->rpm_port; // Get new propeller taken from message, use only one propeller

    simulation.nOde.set(simulation.pNomoto); 
}

void Ros2Simulation::read_callback()
{   
    Ros2Simulation::paramROS2 pROS2 = Ros2Simulation::read(config_file);
    simulation.pNomoto = simulation.nOde.read(pROS2.equ_file);
    simulation.nOde.set(simulation.pNomoto);
}

void Ros2Simulation::sim_callback()
{
    resSim = simulation.runSim(); // Run the integration

    auto n = static_cast<int>(resSim.time.size());
  
    // Get last entry of integration result vector
    double new_x = resSim.x_pos[n-1]; 
    double new_y = resSim.y_pos[n-1]; 
    double new_yaw = resSim.yaw[n-1] ; 
    
    double new_x_v = resSim.x_vel[n-1];
    double new_y_v = resSim.y_vel[n-1];
    double new_yaw_rate = resSim.yaw_rate[n-1]; 
    
    // Last integration result will be start for new integration
    simulation.pSim.initX = new_x; 
    simulation.pSim.initY = new_y; 
    simulation.pSim.initYaw = new_yaw; 

    simulation.pSim.initXVelocity = new_x_v; 
    simulation.pSim.initYVelocity = new_y_v; 
    simulation.pSim.initYawRate = new_yaw_rate; 
    
    // Setup pose position message
    pose_message.pose.position.x = new_x; 
    pose_message.pose.position.y = new_y; 
    pose_message.pose.position.z = 0.0;

    // Setup pose orientation message
    tf2::Quaternion quaternion_tf2;
    double new_yaw_rad = new_yaw * (3.141592653589793238463 / 180);
    quaternion_tf2.setRPY(0.0, 0.0, new_yaw_rad); // Set new orientation (roll and pitch are zero in this model)
    quaternion_tf2.normalize();
    pose_message.pose.orientation.w = quaternion_tf2.getW();
    pose_message.pose.orientation.x = quaternion_tf2.getX();
    pose_message.pose.orientation.y = quaternion_tf2.getY();
    pose_message.pose.orientation.z = quaternion_tf2.getZ();

    pose_message.header.frame_id = frame_id;
    pose_message.header.stamp = this->get_clock()->now();

    publisher_pose->publish(pose_message);

    path_message.poses.push_back(pose_message);
    path_message.header.frame_id =  frame_id;
    path_message.header.stamp = this->get_clock()->now();
    publisher_path->publish(path_message);
}

  

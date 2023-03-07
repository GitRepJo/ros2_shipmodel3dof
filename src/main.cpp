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
/**	
  @name main.cpp
	@brief Spin the ROS2 shipmodel simulation
	@author Jonas Mahler
	@date 03.2022
*/

#include <ros2_sim.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2Simulation>());
  rclcpp::shutdown();
  return 0;
}
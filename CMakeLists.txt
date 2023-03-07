cmake_minimum_required(VERSION 3.5)
project(shipmodel3dof)

set(CMAKE_C_STANDARD 99)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.45.0 COMPONENTS math)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ella_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED )
find_package(tf2 REQUIRED )
find_package(tf2_ros REQUIRED )

add_executable(${PROJECT_NAME} src/main.cpp src/odeint_sim.cpp src/ros2_sim.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs ella_interfaces geometry_msgs nav_msgs yaml-cpp ament_index_cpp tf2
tf2_ros)

target_include_directories(${PROJECT_NAME} PRIVATE include/)

target_link_libraries(${PROJECT_NAME} yaml-cpp )

file(COPY config/config.yaml DESTINATION ./ )

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${CMAKE_SOURCE_DIR}/yaml-cpp/include)

install(TARGETS
${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

install(FILES
test/ros2/expected_data.yaml test/ros2/input_data.yaml test/ros2/simulation_launch_testing.py
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch config test/ros2
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test/nomoto)
  find_package(launch_testing_ament_cmake)  
  add_launch_test(test/ros2/simulation_launch_testing.py) 
endif()
ament_package()
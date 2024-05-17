cmake_minimum_required(VERSION 3.8)
project(drive_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(drone_simple_movement_node src/drone_simple_movement.cpp)
ament_target_dependencies(drone_simple_movement_node rclcpp geometry_msgs)


add_executable(drone_robot_follower_node src/drone_robot_follower.cpp)
ament_target_dependencies(drone_robot_follower_node rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(object_follower_node src/object_follower.cpp)
ament_target_dependencies(object_follower_node rclcpp sensor_msgs cv_bridge geometry_msgs OpenCV)

add_executable(follower_node src/newcode.cpp)
ament_target_dependencies(follower_node rclcpp sensor_msgs cv_bridge geometry_msgs OpenCV)

target_link_libraries(drone_robot_follower_node ${OpenCV_LIBS})
target_link_libraries(object_follower_node ${OpenCV_LIBS})
target_link_libraries(follower_node ${OpenCV_LIBS})




install(TARGETS drone_simple_movement_node drone_robot_follower_node object_follower_node follower_node

  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

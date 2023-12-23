cmake_minimum_required(VERSION 3.5)


project(ros_gz_example_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



#Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(smore_localization_node src/localization_node.cc)
ament_target_dependencies(smore_localization_node rclcpp nav_msgs)

add_executable(smore_convert_velocities_node src/convert_velocities.cc)
ament_target_dependencies(smore_convert_velocities_node rclcpp geometry_msgs)

add_executable(smore_open_loop_controller_node src/open_loop_controller.cc)
ament_target_dependencies(smore_open_loop_controller_node rclcpp geometry_msgs)

add_executable(smore_feedback_controller_node src/feedback_controller.cc)
ament_target_dependencies(smore_feedback_controller_node rclcpp geometry_msgs nav_msgs tf2)


install(TARGETS
  smore_localization_node
  smore_convert_velocities_node
  smore_open_loop_controller_node
  smore_feedback_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

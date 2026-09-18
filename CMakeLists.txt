cmake_minimum_required(VERSION 3.5)
project(localization_msgs_tools)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pose_to_tf_node SHARED src/pose_to_tf.cpp)
ament_auto_add_library(with_covariance_node SHARED src/with_covariance.cpp)

# auto-executable from nodes
rclcpp_components_register_node(pose_to_tf_node
  PLUGIN "localization_msgs_tools::Pose2TF"
  EXECUTABLE pose_to_tf)

rclcpp_components_register_node(with_covariance_node
  PLUGIN "localization_msgs_tools::WithCovariance"
  EXECUTABLE with_covariance)


install(DIRECTORY example
        DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

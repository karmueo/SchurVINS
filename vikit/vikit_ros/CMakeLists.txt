cmake_minimum_required(VERSION 3.22)
project(vikit_ros)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/output_helper.cpp
  src/img_type_conversion.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC cv_bridge minkindr_vendor rclcpp sensor_msgs svo_cmake tf2_ros vikit_common visualization_msgs)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge minkindr_vendor rclcpp sensor_msgs svo_cmake tf2_ros vikit_common visualization_msgs Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

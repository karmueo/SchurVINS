cmake_minimum_required(VERSION 3.22)
project(svo_ceres_backend)
find_package(ament_cmake REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_vio_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include(SvoSetup)
find_package(Ceres 2.2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ceres_backend_interface.cpp
  src/estimator.cpp
  src/error_interface.cpp
  src/homogeneous_point_error.cpp
  src/homogeneous_point_parameter_block.cpp
  src/homogeneous_point_local_parameterization.cpp
  src/imu_error.cpp
  src/local_parameterization_additional_interfaces.cpp
  src/map.cpp
  src/marginalization_error.cpp
  src/pose_error.cpp
  src/pose_parameter_block.cpp
  src/pose_local_parameterization.cpp
  src/relative_pose_error.cpp
  src/speed_and_bias_error.cpp
  src/speed_and_bias_parameter_block.cpp
  src/motion_detector.cpp
  src/outlier_rejection.cpp
  src/ceres_backend_publisher.cpp
  src/map_alignment.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs geometry_msgs minkindr_vendor svo svo_cmake svo_common svo_vio_common)
target_link_libraries(${PROJECT_NAME} PUBLIC Ceres::ceres)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_backend_id test/test_backend_id.cpp)
  target_link_libraries(test_backend_id ${PROJECT_NAME})
  ament_add_gtest(test_estimator test/test_estimator.cpp)
  target_link_libraries(test_estimator ${PROJECT_NAME})
  ament_add_gtest(test_homogeneous_point_error test/test_homogeneous_point_error.cpp)
  target_link_libraries(test_homogeneous_point_error ${PROJECT_NAME})
  ament_add_gtest(test_map test/test_map.cpp)
  target_link_libraries(test_map ${PROJECT_NAME})
  ament_add_gtest(test_marginalization test/test_marginalization.cpp)
  target_link_libraries(test_marginalization ${PROJECT_NAME})
  ament_add_gtest(test_pose_error test/test_pose_error.cpp)
  target_link_libraries(test_pose_error ${PROJECT_NAME})
  ament_add_gtest(test_pose_local_parameterization test/test_pose_local_parameterization.cpp)
  target_link_libraries(test_pose_local_parameterization ${PROJECT_NAME})
  ament_add_gtest(test_reprojection_error test/test_reprojection_error.cpp)
  target_link_libraries(test_reprojection_error ${PROJECT_NAME})
  ament_add_gtest(test_relative_pose_error test/test_relative_pose_error.cpp)
  target_link_libraries(test_relative_pose_error ${PROJECT_NAME})
  ament_add_gtest(test_speed_and_bias_error test/test_speed_and_bias_error.cpp)
  target_link_libraries(test_speed_and_bias_error ${PROJECT_NAME})
  ament_add_gtest(test_imu_error test/test_imu_error.cpp)
  target_link_libraries(test_imu_error ${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs minkindr_vendor svo svo_cmake svo_common svo_vio_common Eigen3 glog gflags yaml-cpp OpenCV Threads Ceres)
ament_package()

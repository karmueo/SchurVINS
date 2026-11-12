cmake_minimum_required(VERSION 3.22)
project(svo_test_utils)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(vikit_cameras REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/test_utils.cpp
  src/imu_simulation.cpp
  src/synthetic_dataset.cpp
  src/simple_raytracer.cpp
  src/trajectory_generator.cpp
  src/point_cloud_generator.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp cv_bridge minkindr_vendor svo_cmake svo_common vikit_cameras vikit_common vikit_ros)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp cv_bridge minkindr_vendor svo_cmake svo_common vikit_cameras vikit_common vikit_ros Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

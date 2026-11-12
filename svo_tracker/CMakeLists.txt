cmake_minimum_required(VERSION 3.22)
project(svo_tracker)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_direct REQUIRED)
find_package(svo_test_utils REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/feature_tracking_types.cpp
  src/feature_tracking_utils.cpp
  src/feature_tracking_viz.cpp
  src/feature_tracker.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp cv_bridge minkindr_vendor svo_cmake svo_common svo_direct svo_test_utils)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp cv_bridge minkindr_vendor svo_cmake svo_common svo_direct svo_test_utils Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

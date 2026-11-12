cmake_minimum_required(VERSION 3.22)
project(svo_vio_common)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/vio_common.cpp
  src/test_utils.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC geometry_msgs minkindr_vendor std_msgs svo_cmake svo_common)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs minkindr_vendor std_msgs svo_cmake svo_common Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

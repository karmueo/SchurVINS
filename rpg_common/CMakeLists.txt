cmake_minimum_required(VERSION 3.22)
project(rpg_common)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/fs.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC cv_bridge minkindr_vendor svo_cmake)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge minkindr_vendor svo_cmake Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

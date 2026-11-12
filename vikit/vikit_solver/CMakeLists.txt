cmake_minimum_required(VERSION 3.22)
project(vikit_solver)
find_package(ament_cmake REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/robust_cost.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC minkindr_vendor svo_cmake)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(minkindr_vendor svo_cmake Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

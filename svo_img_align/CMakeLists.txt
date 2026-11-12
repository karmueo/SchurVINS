cmake_minimum_required(VERSION 3.22)
project(svo_img_align)
find_package(ament_cmake REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_direct REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(vikit_solver REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC minkindr_vendor svo_cmake svo_common svo_direct vikit_ros vikit_solver)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(minkindr_vendor svo_cmake svo_common svo_direct vikit_ros vikit_solver Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

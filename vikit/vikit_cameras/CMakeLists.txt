cmake_minimum_required(VERSION 3.22)
project(vikit_cameras)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(vikit_common REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/camera_factory.cpp
  src/ncamera.cpp
  src/camera_geometry_base.cpp
  src/camera_yaml_serialization.cpp
  src/equidistant_fisheye_geometry.cpp
  src/equidistant_fisheye_projection.cpp
  src/omni_geometry.cpp
  src/omni_projection.cpp
  src/ncamera_yaml_serialization.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC cv_bridge minkindr_vendor svo_cmake vikit_common)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(equidistant_ego test/test_equidistant_ego.cpp)
  target_link_libraries(equidistant_ego ${PROJECT_NAME})
  ament_add_gtest(${PROJECT_NAME}_tests
  test/test_cameras.cpp
)
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge minkindr_vendor svo_cmake vikit_common Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

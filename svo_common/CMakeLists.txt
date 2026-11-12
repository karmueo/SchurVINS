cmake_minimum_required(VERSION 3.22)
project(svo_common)
find_package(ament_cmake REQUIRED)
find_package(fast_neon_vendor REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(vikit_cameras REQUIRED)
find_package(vikit_common REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/frame.cpp
  src/point.cpp
  src/local_feature.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC fast_neon_vendor minkindr_vendor svo_cmake vikit_cameras vikit_common)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/test_feature_wrapper.cpp
)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  ament_add_gtest(container-helpers-test
  test/test_container_helpers.cpp
)
  target_link_libraries(container-helpers-test ${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fast_neon_vendor minkindr_vendor svo_cmake vikit_cameras vikit_common Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

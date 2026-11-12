cmake_minimum_required(VERSION 3.22)
project(vikit_common)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(svo_cmake REQUIRED)
include(SvoSetup)

add_library(${PROJECT_NAME} SHARED
  src/homography.cpp
  src/math_utils.cpp
  src/performance_monitor.cpp
  src/sample.cpp
  src/user_input_thread.cpp
  src/vision.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC cv_bridge minkindr_vendor rclcpp svo_cmake)
add_executable(vikit_test_timer test/test_timer.cpp)
target_link_libraries(vikit_test_timer PRIVATE ${PROJECT_NAME})
install(TARGETS vikit_test_timer DESTINATION lib/${PROJECT_NAME})
add_executable(vikit_test_common test/test_common.cpp)
target_link_libraries(vikit_test_common PRIVATE ${PROJECT_NAME})
install(TARGETS vikit_test_common DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(homography test/test_homography.cpp)
  target_link_libraries(homography ${PROJECT_NAME})
  ament_add_gtest(math_utils test/test_math_utils.cpp)
  target_link_libraries(math_utils ${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge minkindr_vendor rclcpp svo_cmake Eigen3 glog gflags yaml-cpp OpenCV Threads)
ament_package()

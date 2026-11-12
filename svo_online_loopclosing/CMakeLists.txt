cmake_minimum_required(VERSION 3.22)
project(svo_online_loopclosing)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dbow2_vendor REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rpg_common REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_pgo REQUIRED)
include(SvoSetup)
find_package(opengv REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/geometric_verification.cpp
  src/loop_closing.cpp
  src/read_file.cpp
  src/map_alignment.cpp
  src/bow.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp cv_bridge dbow2_vendor minkindr_vendor rclcpp rpg_common svo_cmake svo_common svo_pgo)
target_link_libraries(${PROJECT_NAME} PUBLIC opengv)
add_executable(createvoc test/createvoc.cpp)
target_link_libraries(createvoc PRIVATE ${PROJECT_NAME})
install(TARGETS createvoc DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp cv_bridge dbow2_vendor minkindr_vendor rclcpp rpg_common svo_cmake svo_common svo_pgo Eigen3 glog gflags yaml-cpp OpenCV Threads opengv)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vocabulary test/test_vocabulary.cpp)
  target_link_libraries(test_vocabulary ${PROJECT_NAME})
endif()
ament_package()

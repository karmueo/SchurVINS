cmake_minimum_required(VERSION 3.22)
project(svo)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fast_neon_vendor REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rpg_common REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_direct REQUIRED)
find_package(svo_global_map REQUIRED)
find_package(svo_img_align REQUIRED)
find_package(svo_online_loopclosing REQUIRED)
find_package(svo_test_utils REQUIRED)
find_package(svo_tracker REQUIRED)
find_package(svo_vio_common REQUIRED)
find_package(vikit_cameras REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(vikit_solver REQUIRED)
include(SvoSetup)
find_package(opengv REQUIRED)
find_package(GTSAM 4.2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/schur_vins.cpp
  src/initialization.cpp
  src/reprojector.cpp
  src/imu_handler.cpp
  src/stereo_triangulation.cpp
  src/io.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp cv_bridge fast_neon_vendor minkindr_vendor rclcpp rpg_common svo_cmake svo_common svo_direct svo_global_map svo_img_align svo_online_loopclosing svo_test_utils svo_tracker svo_vio_common vikit_cameras vikit_common vikit_ros vikit_solver)
target_link_libraries(${PROJECT_NAME} PUBLIC opengv gtsam)
target_compile_definitions(${PROJECT_NAME} PRIVATE SVO_USE_GTSAM)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test-frame test/test_frame.cpp)
  target_link_libraries(${PROJECT_NAME}-test-frame ${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp cv_bridge fast_neon_vendor minkindr_vendor rclcpp rpg_common svo_cmake svo_common svo_direct svo_global_map svo_img_align svo_online_loopclosing svo_test_utils svo_tracker svo_vio_common vikit_cameras vikit_common vikit_ros vikit_solver Eigen3 glog gflags yaml-cpp OpenCV Threads opengv GTSAM)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_array_initialization test/test_array_initialization.cpp)
  target_link_libraries(test_array_initialization ${PROJECT_NAME})
endif()
ament_package()

cmake_minimum_required(VERSION 3.22)
project(svo_global_map)
find_package(ament_cmake REQUIRED)
find_package(minkindr_vendor REQUIRED)
find_package(rpg_common REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_vio_common REQUIRED)
include(SvoSetup)
find_package(GTSAM 4.2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gtsam/gtsam_optimizer.cpp
  src/gtsam/graph_manager.cpp
  src/global_map.cpp
  src/null_stream.cpp
)
svo_target(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} PUBLIC minkindr_vendor rpg_common svo_cmake svo_common svo_vio_common)
target_link_libraries(${PROJECT_NAME} PUBLIC gtsam)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(minkindr_vendor rpg_common svo_cmake svo_common svo_vio_common Eigen3 glog gflags yaml-cpp OpenCV Threads GTSAM)
ament_package()

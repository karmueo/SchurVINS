cmake_minimum_required(VERSION 3.22)
project(svo_benchmarking)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(svo_ceres_benchmarking PACKAGE_DIR src/svo_ceres_benchmarking)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)
install(DIRECTORY dataset_tools/ DESTINATION share/${PROJECT_NAME}/dataset_tools USE_SOURCE_PERMISSIONS)
install(DIRECTORY experiments/ DESTINATION share/${PROJECT_NAME}/experiments USE_SOURCE_PERMISSIONS)
install(PROGRAMS
  dataset_tools/python/asl_groundtruth_to_pose.py
  dataset_tools/python/associate_timestamps.py
  dataset_tools/python/bag_to_image.py
  dataset_tools/python/bag_to_imu.py
  dataset_tools/python/bag_to_pose.py
  dataset_tools/python/change_image_path.py
  dataset_tools/python/combine_images.py
  dataset_tools/python/prepend_id.py
  dataset_tools/python/restamp_bag.py
  dataset_tools/python/strip_gt_id.py
  dataset_tools/python/swap_stamped_imu_meas.py
  dataset_tools/python/transform_trajectory.py
  scripts/analyze_imu_integration_dynamic.py
  scripts/analyze_imu_integration_static.py
  scripts/analyze_imu_properties_static.py
  scripts/analyze_lc_timing.py
  scripts/analyze_vio_timing.py
  scripts/bag_eval_scripts/analyze_stationary.py
  scripts/bag_eval_scripts/cal_traj_len.py
  scripts/benchmark.py
  scripts/compute_hand_eye_transformation.py
  scripts/convert_old_fmt.py
  scripts/evaluate.py
  scripts/evaluate_all.py
  scripts/organize_results.py
  scripts/organize_results_mul.py
  scripts/reference_report.py
  scripts/ros_node.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dataset_tools test/test_dataset_tools.py TIMEOUT 120)
  ament_add_pytest_test(test_rosbag2_tools test/test_rosbag2_tools.py TIMEOUT 120)
  ament_add_pytest_test(test_reference_report test/test_reference_report.py TIMEOUT 120)
endif()
install(PROGRAMS dataset_tools/prepare_dataset_monocular.sh dataset_tools/prepare_dataset_stereo.sh dataset_tools/generate_ground_truth.sh scripts/analyze_all_euroc.sh DESTINATION lib/${PROJECT_NAME})
ament_package()

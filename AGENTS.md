# 仓库贡献指南

## 项目结构与模块组织

SchurVINS 基于 SVO2.0 扩展，当前使用 ROS2 Jazzy 和 ament 构建。核心状态估计代码位于 `svo/`，包括 `svo/src/schur_vins.cpp`。前端功能分布在 `svo_direct/`、`svo_tracker/` 和 `svo_img_align/`；优化与建图相关包包括 `svo_ceres_backend/`、`svo_pgo/` 和 `svo_global_map/`。共享类型与工具位于 `svo_common/`、`svo_vio_common/`、`vikit/` 和 `rpg_common/`；固定版本依赖封装位于 `vendor/`。

各包通常将实现代码放在 `src/`，公共头文件放在 `include/`，测试放在 `test/`。ROS2 节点、组件、Python launch、YAML 参数和 RViz2 配置位于 `svo_ros/`。配套文档和评估工具分别见 `doc/` 与 `svo_benchmarking/`；rqt 界面位于 `rqt_svo/`。

## 构建、测试与开发命令

目标环境为 Ubuntu 24.04、ROS2 Jazzy、C++17 和 Python 3.12。系统依赖安装、固定依赖与资源说明见 [Jazzy 运行指南](doc/ROS2_JAZZY.md)，贡献约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。以下命令假设仓库位于 `~/schurvins_ws/src/SchurVINS`；实际路径可相应调整。

```bash
# 在仅加载 Jazzy 的新终端中准备固定依赖和官方词典。
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
export SCHURVINS_DEPENDENCY_DIR="$HOME/.cache/schurvins/dependencies"
cd ~/schurvins_ws/src/SchurVINS
/usr/bin/python3 scripts/prepare_dependencies.py "$SCHURVINS_DEPENDENCY_DIR"
/usr/bin/python3 scripts/prepare_resources.py "$HOME/.cache/schurvins/resources"

# 构建默认双目 SchurVINS + IMU 配置。
cd ~/schurvins_ws
colcon --log-base log-default build --base-paths src/SchurVINS \
  --build-base build-default --install-base install-default \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DUSE_SCHUR_VINS=ON -DUSE_LOOP_CLOSING=ON -DUSE_GLOBAL_MAP=OFF
source install-default/setup.bash
colcon --log-base log-default test --build-base build-default --install-base install-default
colcon test-result --test-result-base build-default --verbose
```

依赖已构建且未改变依赖关系时，可在上述构建命令中增加 `--packages-select svo` 仅重建核心包；核心包测试可在测试命令中增加同一选项。使用系统 Eigen 3.4、OpenCV 4.6、Ceres 2.2、yaml-cpp 0.8、Jazzy GTSAM 4.2 和 OpenGV；不要混入用户安装的 OpenCV 或 Conda Python。上面的 `OpenCV_DIR` 适用于 amd64，其他架构使用对应系统路径。

默认、全功能和传统 SVO 配置分别使用独立的 build/install/log 目录。全功能配置开启 `USE_GLOBAL_MAP=ON`；单目、纯视觉和 array 使用 `USE_SCHUR_VINS=OFF` 的传统配置，需要全局地图时另行开启对应选项。切换配置应使用新终端且只加载 Jazzy，再加载所选安装目录；具体命令见运行指南。

## 运行与数据回放

运行终端均须先加载 Jazzy 和所选配置的 `install-*/setup.bash`。例如，启动 EuRoC 双目惯性节点，并在另一个已加载环境的终端回放原生 ROS2 MCAP 或 SQLite3 数据：

```bash
ros2 launch svo_ros euroc_vio_stereo.launch.py output_dir:="$HOME/schurvins-results/euroc"
# 在另一终端执行；数据需包含 /cam0/image_raw、/cam1/image_raw 和 /imu0。
ros2 bag play /absolute/path/to/ros2_euroc --clock --rate 0.25
```

使用 `ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share svo_ros)/rviz_config_vio.rviz"` 查看轨迹，使用 `ros2 run rqt_svo rqt_svo` 启动界面。参数文件使用 `ros__parameters` 和点号层级；通过 ament 索引访问已安装资源，算法和标定参数启动后只读。输出写入显式 `output_dir` 下的独立运行目录。

EGO 数据的完整回放优先使用 `scripts/accept_ego.py`，参数和示例见运行指南：每次使用新的输出目录，从零偏移、0.25 倍速开始，通过后另跑 1 倍速性能测试。等待 DDS 匹配后开始回放，结束后等待解码和估计队列排空，再发送 `q` 刷新输出；自动验收入口负责这些步骤。源数据、标定与参考轨迹保持只读，参考轨迹含其他 VIO 先验，不能视为独立真值。

## 代码风格与命名约定

使用 C++17 和 Python 3.12，并遵循相邻代码的风格：通常采用两空格缩进、花括号独占一行；文件名和变量使用 `snake_case`，类型使用 `PascalCase`，方法使用 `camelCase`，成员变量以下划线结尾。保留现有文件头中的许可证声明。共享编译设置位于 `svo_cmake/cmake/Modules/SvoSetup.cmake`；使用目标级 ament 依赖，导出并安装公共头文件、库和运行资源，确保工具可从源码目录外运行。仓库未提供统一的代码格式化或静态检查工具配置。

## 测试指南

C++ 测试使用 `ament_add_gtest`，Python 测试使用 `ament_add_pytest_test`；文件命名为 `test/test_<feature>.cpp` 或 `.py`，在所属包的 `CMakeLists.txt` 中注册。执行相关数值、生命周期及数据集检查；依赖外部数据或显示环境而未执行的用例须单独记录，不能将仅编译通过或没有断言的 fixture 作为功能验收。项目未规定覆盖率门槛。

修改状态估计相关代码时，报告数据集序列、标定配置、起始偏移、回放速率、初始化和跟踪状态、精度与运行耗时；EuRoC 论文复现的起始偏移参照 README，EGO 主验收使用零偏移。修改 Ceres 接口时保留自定义流形的更新方向、布局及尺度，并验证雅可比。涉及传统或全功能配置时，在对应配置中执行验证。实际验收结果见 `doc/MIGRATION_VALIDATION.md`；未执行的验证必须明确说明。

## 提交与拉取请求指南

现有提交使用简短的描述性标题，例如 `Update README.md`，尚未建立正式的提交前缀约定。每次提交应聚焦单一主题。拉取请求应说明所解决的问题、受影响的包、验证命令及结果，并关联相关议题。若有助于说明行为变化，请附上轨迹图或 RViz2 截图。保留无关工作区修改；不要提交下载的数据集或词典、生成的日志与结果、build/install 目录，或源标定文件的修改。

# rpg_svo_pro

本仓库包含 **SVO Pro**，即苏黎世大学机器人与感知实验室（RPG）在过去几年中开发的最新版本半直接视觉里程计（SVO）。
SVO 最初是一个快速、通用的视觉前端，详见 [SVO 论文（TRO-17）](http://rpg.ifi.uzh.ch/docs/TRO17_Forster-SVO.pdf)。此后，各种研究和工业项目为其集成了不同的扩展功能。
SVO Pro 支持不同的[相机模型](http://rpg.ifi.uzh.ch/docs/ICRA16_Zhang.pdf)、[主动曝光控制](http://rpg.ifi.uzh.ch/docs/ICRA17_Zhang.pdf)、基于滑动窗口的后端，以及带回环检测的全局束调整。

本仓库提供以下功能：

* 视觉里程计：最新版本的 SVO，支持单目或双目配置的透视相机和鱼眼/折反射相机，还包含主动曝光控制。
* 视觉惯性里程计：SVO 前端 + 视觉惯性滑动窗口优化后端（基于 [OKVIS](https://github.com/ethz-asl/okvis) 修改）。
* 视觉惯性 SLAM：SVO 前端 + 视觉惯性滑动窗口优化后端 + 经过全局束调整的地图（使用 [iSAM2](https://gtsam.org/)）。借助 iSAM2，全局地图可实时更新，并以帧率进行定位。
* 带回环检测的视觉惯性 SLAM：通过 [DBoW2](https://github.com/dorian3d/DBoW2) 检测的回环被集成到全局束调整中。此外还提供位姿图优化，作为全局束调整的轻量替代方案。

下图展示了视觉惯性 SLAM 流程在 EuRoC 数据集上的运行示例（绿色点表示滑动窗口；蓝色点表示 iSAM2 地图）：

![](./doc/images/v102_gm.gif)

SVO Pro 及其扩展已用于支持 RPG 的多个项目，例如近期的[多相机 SLAM](http://rpg.ifi.uzh.ch/docs/ICRA20_Kuo.pdf)、[用于视觉 SLAM 的体素地图](http://rpg.ifi.uzh.ch/docs/ICRA20_Muglikar.pdf)以及[将全局位置测量紧耦合到 VIO 中](http://rpg.ifi.uzh.ch/docs/IROS20_Cioffi.pdf)的研究。我们希望这些工作能够促进 SLAM 和空间感知领域的研究与应用。

## 许可证

代码采用 GPLv3 许可证。商业用途请联系 `sdavide [at] ifi [dot] uzh [dot] ch`。

视觉惯性后端基于 OKVIS 修改，相关文件的开头保留了原许可证。

## 引用

如果在学术研究中使用本代码，请引用：

* Christian Forster, Matia Pizzoli, Davide Scaramuzza. SVO: Fast Semi-Direct Monocular Visual Odometry. ICRA, 2014. [bibtex](./doc/bib/Forster14icra.bib)
* Christian Forster, Zichao Zhang, Michael Gassner, Manuel Werlberger, Davide Scaramuzza. SVO: Semi-Direct Visual Odometry for Monocular and Multi-Camera Systems. TRO, 2017. [bibtex](./doc/bib/Forster17tro.bib)

此外，请根据使用的具体扩展功能引用以下论文：

* *鱼眼/折反射相机扩展*：Zichao Zhang, Henri Rebecq, Christian Forster, Davide Scaramuzza. Benefit of Large Field-of-View Cameras for Visual Odometry. ICRA, 2016. [bibtex](./doc/bib/Zhang16icra.bib)
* *亮度/曝光补偿*：Zichao Zhang, Christian Forster, Davide Scaramuzza. Active Exposure Control for Robust Visual Odometry in HDR Environments. ICRA, 2017. [bibtex](./doc/bib/Zhang17icra.bib)
* *基于 Ceres 的优化后端*：Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart, Paul Timothy Furgale. Keyframe-based visual–inertial odometry using nonlinear optimization. IJRR, 2015. [bibtex](./doc/bib/Leutenegger15ijrr.bib)
* *基于 iSAM2 的全局地图*：Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, Frank Dellaert. iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree. IJRR, 2012. [bibtex](./doc/bib/Kaess12ijrr.bib)
* *回环检测*：Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. TRO, 2012. [bibtex](./doc/bib/Galvez12tro.bib)

近期使用 SVO Pro 的论文包括：

* *多相机 SLAM*：Juichung Kuo, Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Redesigning SLAM for Arbitrary Multi-Camera Systems. ICRA, 2020. [bibtex](./doc/bib/Kuo20icra.bib)
* *用于视觉 SLAM 的体素地图*：Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Voxel Map for Visual SLAM. ICRA, 2020. [bibtex](./doc/bib/Muglikar20icra.bib)
* *将全局位置测量紧耦合到 VIO 中*：Giovanni Cioffi, Davide Scaramuzza. Tightly-coupled Fusion of Global Positional Measurements in Optimization-based Visual-Inertial Odometry. IROS, 2020. [bibtex](./doc/bib/Cioffi20iros.bib)

## 安装

代码已在以下环境中测试：

* Ubuntu 18.04 和 ROS Melodic
* Ubuntu 20.04 和 ROS Noetic

### 安装依赖

如果尚未安装，请安装 [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) 和 [vcstools](https://github.com/dirk-thomas/vcstool)。根据操作系统运行相应命令：

```sh
# 适用于 Ubuntu 18.04 + Melodic
sudo apt-get install python-catkin-tools python-vcstool
```

或：

```sh
# 适用于 Ubuntu 20.04 + Noetic
sudo apt-get install python3-catkin-tools python3-vcstool python3-osrf-pycommon
```

安装系统依赖和 Ceres Solver 的依赖：

```sh
# 系统依赖
sudo apt-get install libglew-dev libopencv-dev libyaml-cpp-dev 
# Ceres 依赖
sudo apt-get install libblas-dev liblapack-dev libsuitesparse-dev
```

### 克隆和编译

创建工作空间并克隆代码（`ROS-DISTRO` 为 `melodic` 或 `noetic`）：

```sh
mkdir svo_ws && cd svo_ws
# 指定 Eigen 路径的原因见下文
catkin config --init --mkdirs --extend /opt/ros/<ROS-DISTRO> --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
cd src
git clone git@github.com:uzh-rpg/rpg_svo_pro_open.git
vcs-import < ./rpg_svo_pro_open/dependencies.yaml
touch minkindr/minkindr_python/CATKIN_IGNORE
# 下载用于地点识别的词汇库
cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
cd ../../..
```

接下来可以选择两种构建方式：

1. 不构建全局地图（**前端 + 滑动窗口后端 + 回环检测/位姿图**）

   ```sh
   catkin build
   ```

2. 使用 iSAM2 构建全局地图（**全部功能**）

    首先，启用全局地图功能：

    ```sh
    rm rpg_svo_pro_open/svo_global_map/CATKIN_IGNORE
    ```

    并在 `svo_cmake/cmake/Modules/SvoSetup.cmake` 中设置：

    ```cmake
    SET(USE_GLOBAL_MAP TRUE)
    ```

    接着克隆 GTSAM：

    ```sh
    git clone --branch 4.0.3 git@github.com:borglab/gtsam.git
    ```

    然后稍作修改 GTSAM 的编译选项：

    ```cmake
    # 1. gtsam/CMakelists.txt：使用系统 Eigen
    -option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
    +option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" ON)
    # 2. gtsam/cmake/GtsamBuildTypes：禁用 AVX 指令集
    # 在 `list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-march=native")` 行下方添加
    list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-mno-avx")
    ```

    > 使用相同版本的 Eigen 有助于避免[内存问题](https://github.com/ethz-asl/eigen_catkin/wiki/Eigen-Memory-Issues)。根据我们的经验，禁用 `avx` 指令集也有助于解决某些段错误（但这可能取决于操作系统和硬件）。

    最后构建整个工作空间：

    ```sh
    # 构建 GTSAM 可能需要一些时间
    catkin build
    ```

## 使用说明

* 入门：运行处理流程
    * [视觉前端](./doc/frontend/visual_frontend.md)
    * [视觉惯性里程计](./doc/vio.md)
    * [VIO + 全局地图](./doc/global_map.md)
* [基准测试](./doc/benchmarking.md)
* [相机和传感器标定](./doc/calibration.md)
* [已知问题和可能的改进](./doc/known_issues_and_improvements.md)

## 故障排查

0. **修改配置后出现奇怪的构建问题**：修改 CMake 文件（`CMakeLists.txt` 或 `*.cmake`）后，建议执行以下操作以确保更改生效：

    * 清理工作空间（在工作空间根目录运行 `catkin clean --all`），然后重新构建（`catkin build`）；
    * 或运行 `catkin build --force-cmake`。

    <details>
      <summary>详细说明</summary>
      Catkin tools 可以检测 CMake 文件的更改，并仅重新构建受影响的文件。但由于这是一个多包项目，某些更改可能无法按预期被检测到。例如，修改 `svo_cmake/cmake/Modules/SvoSetup.cmake` 中的构建选项会影响所有包，但可能不会自动重新编译（因为各个包中的文件没有改变）。此外，还需要保持不同包之间的链接设置（例如库版本）和编译选项一致。因此，除非熟悉具体的编译过程，否则重新构建整个工作空间最为稳妥。在大多数情况下，`catkin build --force-cmake` 也应该有效。
    </details>

1. **与 OpenCV 相关的编译或链接错误**：在各个包（`rpg_common`、`svo_ros`、`svo_direct`、`vikit/vikit_common` 和 `svo_online_loopclosing`）的 `CMakeLists.txt` 文件中找到 `find_package(OpenCV REQUIRED)`，将其替换为：

   ```cmake
   # Ubuntu 18.04 + Melodic
   find_package(OpenCV 3 REQUIRED)
   # Ubuntu 20.04 + Noetic
   find_package(OpenCV 4 REQUIRED)
   ```

   <details>
     <summary>详细说明</summary>
     首先，在 Ubuntu 18.04 上 ROS 基于 OpenCV 3 构建，而在 Ubuntu 20.04 上基于 OpenCV 4 构建。由于 `svo_ros` 需要将所有内容与 ROS 链接，因此最好保持 SVO 链接的 OpenCV 版本与 ROS 一致。其次，如果只安装了默认的 OpenCV，原有的 `CMakeLists.txt` 文件可以正常工作。但如果安装了自定义版本的 OpenCV（例如从源码安装），建议如上所述显式指定要使用的 OpenCV 版本，即 ROS 使用的版本。
   </details>

2. **PointCloud2 的可视化问题**：在 Ubuntu 20.04 上，使用 `Points` 在 RVIZ 中显示 `PointCloud2` 似乎存在[问题](https://github.com/ros-visualization/rviz/issues/1508)。默认情况下，我们使用其他可视化类型。如果需要自定义可视化，请留意这一问题。

3. **启用回环检测后流程崩溃**：如果流程在调用 `svo::loadVoc()` 时崩溃，请检查是否按照上文下载了词汇库文件：

    ```sh
    cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
    ```

4. **编译时 Eigen 版本不一致**：整个项目应使用相同的 Eigen 版本（由于项目也使用 ROS，应当使用系统 Eigen）。检查 `eigen_catkin` 和 `gtsam` 是否找到了相同版本的 Eigen：

    ```sh
    # 检查 eigen_catkin
    catkin build eigen_catkin --force-cmake --verbose
    # 检查 gtsam
    catkin build gtsam --force-cmake --verbose
    ```

    <details>
         <summary>详细说明</summary>
    在项目中使用 Eigen 时，一个常见问题是不同的库基于不同版本的 Eigen 编译。SVO 使用 eigen_catkin (https://github.com/ethz-asl/eigen_catkin) 来保持 Eigen 版本一致；在 Ubuntu 18.04 和 20.04 上，应使用系统版本（位于 /usr/include 下）。GTSAM 通过自定义的 CMake 文件 (https://github.com/borglab/gtsam/blob/develop/cmake/FindEigen3.cmake#L66) 查找系统 Eigen。该文件首先搜索 `/usr/local/include`，其中可能包含手动安装的 Eigen 版本。因此，我们在配置工作空间时显式指定 `EIGEN_INCLUDE_PATH`，强制 GTSAM 查找系统 Eigen。如果仍然遇到 Eigen 版本不一致的问题，首先应检查是否仍在使用不同版本的 Eigen。
    </details>

## 致谢

感谢 Simon Klenk、Manasi Muglikar、Giovanni Cioffi 和 Javier Hidalgo-Carrió 对开源代码提供的宝贵帮助和建议。

这项工作得益于 RPG 众多贡献者的努力。除了上述论文中列出的作者，Titus Cieslewski 和 Henri Rebecq 对视觉前端做出了重要贡献。Jeffrey Delmerico 为将 SVO 应用于不同的真实机器人付出了大量努力，这也帮助改进了整个流程。许多博士生、硕士生和实验室工程师也为代码做出了贡献。

基于 Ceres 的优化后端基于 [Zurich-eye](https://www.wysszurich.uzh.ch/projects/completed-projects/zurich-eye) 开发的代码；Zurich-eye 是一家从 RPG 孵化的公司。Jonathan Huber 是将后端与 SVO 集成的主要贡献者。Kunal Shrivastava（现任 [SUIND](https://suind.com/) 首席执行官）在 RPG 的学期项目和实习期间开发了回环检测模块。基于 iSAM2 的全局地图集成由 Zichao Zhang 开发。

感谢我们在 [Prophesee](https://www.prophesee.ai/) 的合作者指出视觉前端中的多个错误。部分代码是在与 [Huawei](https://www.huawei.com/en/) 合作的资助项目期间开发的。

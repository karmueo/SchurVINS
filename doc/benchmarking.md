# Benchmarking on ROS2 Jazzy

The installed `svo_benchmarking` tools support native MCAP/SQLite3 export, mono/stereo/array offline data, isolated multi-trial runs and local no-scale SE(3) reference evaluation. Follow the [package guide](../svo_benchmarking/README.md) and [dataset format](../svo_benchmarking/dataset_tools/README.md).

Use external data/output directories and the correct build profile. Compilation, numerical fixtures, GUI smoke tests and real-dataset feature validation are separate results; report each explicitly. The [Jazzy operating guide](ROS2_JAZZY.md) defines complete EGO playback counts, rate tests and non-independent reference reporting.

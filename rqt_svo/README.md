# rqt_svo on ROS2 Jazzy

Source the selected install and run:

```bash
ros2 run rqt_svo rqt_svo
```

Alternatively open `rqt` and select the Svo plugin. The namespace field selects `<namespace>/info` and `<namespace>/remote_key`; editing is applied after finishing the field. Start, reset and quit send the existing `s`, `r`, `q` controls.

The plugin uses the rqt context's ROS2 node, installed ament resources, executor-safe message snapshots and GUI-thread updates. Closing the widget stops its timer and destroys only its own subscription/publisher. For automated GUI checks use `QT_QPA_PLATFORM=offscreen` or `xvfb-run`; a successful widget smoke test is separate from real camera validation.

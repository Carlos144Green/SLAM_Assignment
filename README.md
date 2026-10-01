# SLAM Project

<p align="center">
  <a href="https://www.ros.org/">
    <img src="https://img.shields.io/badge/ROS%202-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2">
  </a>
  <a href="https://huggingface.co/datasets/OmniInstrument/SLAM_project">
    <img src="https://img.shields.io/badge/HuggingFace-Dataset-yellow?logo=huggingface&style=for-the-badge">
  </a>
  <a href="https://rerun.io/">
    <img src="https://img.shields.io/badge/Rerun.io-FF3E58?style=for-the-badge&logo=data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBzdGFuZGFsb25lPSJubyI/Pgo8IURPQ1RZUEUgc3ZnIFBVQkxJQyAiLS8vVzNDLy9EVEQgU1ZHIDIwMDEwOTA0Ly9FTiIKICJodHRwOi8vd3d3LnczLm9yZy9UUi8yMDAxL1JFQy1TVkctMjAwMTA5MDQvRFREL3N2ZzEwLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4wIiB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciCiB3aWR0aD0iNDAwLjAwMDAwMHB0IiBoZWlnaHQ9IjQwMC4wMDAwMDBwdCIgdmlld0JveD0iMCAwIDQwMC4wMDAwMDAgNDAwLjAwMDAwMCIKIHByZXNlcnZlQXNwZWN0UmF0aW89InhNaWRZTWlkIG1lZXQiPgoKPGcgdHJhbnNmb3JtPSJ0cmFuc2xhdGUoMC4wMDAwMDAsNDAwLjAwMDAwMCkgc2NhbGUoMC4xMDAwMDAsLTAuMTAwMDAwKSIKZmlsbD0iIzAwMDAwMCIgc3Ryb2tlPSJub25lIj4KPHBhdGggZD0iTTAgMjAwMCBsMCAtMjAwMCAyMDAwIDAgMjAwMCAwIDAgMjAwMCAwIDIwMDAgLTIwMDAgMCAtMjAwMCAwIDAKLTIwMDB6IG0yMjEwIDEwNzIgYzMxIC0yNSAyMSAtNjAgLTI1IC05MCAtMjAgLTEyIC0yMzAgLTEzNiAtNDY4IC0yNzQgLTUxMAotMjk2IC01MDIgLTI5MiAtNTIxIC0zMjAgLTEyIC0xOSAtMTUgLTExMSAtMTggLTUyMCAtNCAtMzk1IC03IC01MDAgLTE4IC01MTMKLTE5IC0yMyAtNjAgLTE1IC04NCAxOCAtMjEgMjggLTIxIDM5IC0yNCA1MjIgLTIgMzMwIDAgNTA3IDggNTMzIDE0IDUyIDIwIDU3CjU5MCAzODcgNDk0IDI4NyA1MTIgMjk2IDU2MCAyNTd6IG0xNzYgLTE3NiBjMTggLTE0IDE5IC0zMiAyMiAtNDYzIDIgLTI0NiAwCi00NzIgLTMgLTUwMiAtOSAtNjggLTIxIC04MCAtMjAwIC0xODYgLTE2MSAtOTUgLTE5NSAtMTIwIC0xOTUgLTE0MyAwIC0yOCA0MQotNjIgNzQgLTYyIDU3IDEgNDE3IDIwNiA0NDggMjU1IDE3IDI4IDE4IDYyIDE4IDUxMiAwIDI5NyA0IDQ5MiAxMCA1MDggOCAyMgoxNSAyNiA0MiAyMyAyMCAtMiAzOCAtMTMgNTEgLTI5IDE5IC0yNiAxOSAtNDcgMjEgLTU0MiAxIC01MDYgMSAtNTE2IC0yMAotNTQ0IC0yMyAtMzEgLTg5IC03NyAtMjU1IC0xNzMgLTEzMCAtNzUgLTE0OSAtOTkgLTEwNiAtMTM2IDU2IC00OCAxMTEgLTI4CjM4OSAxNDMgMTQxIDg3IDEyOCAyNSAxMjggNTk4IDAgNDYzIDEgNDk0IDE4IDUxNSAyNCAyOSA2NCAyMSA4OSAtMTcgMTcgLTI1CjE4IC03MCAyMSAtNTMzIDMgLTQ2OSAyIC01MDcgLTE1IC01NDAgLTEzIC0yNiAtNDQgLTUxIC0xMTYgLTk1IC0yNjEgLTE1OQotMjYyIC0xNjAgLTI2NSAtMTgzIC01IC0zMyAxNiAtNTIgMTI0IC0xMTcgMTM0IC04MCAxNDYgLTExNCA0MiAtMTIzIC02NiAtNQotOTAgNyAtNTU2IDI3NyAtMTgzIDEwNiAtMzQ4IDE5OSAtMzY2IDIwNiAtNDYgMTkgLTc2IC0xIC03NiAtNTIgMCAtNTEgMzUKLTgwIDIzNyAtMTk2IDI5OCAtMTcxIDMzOCAtMjA0IDI3OCAtMjI3IC03MCAtMjcgLTExMiAtMTMgLTMyMCAxMDYgLTgyIDQ2Ci0xNDIgNzQgLTE1NiA3MiAtMzcgLTUgLTQ1IC0zMCAtMzAgLTEwMCAxMiAtNTcgMTEgLTYzIC02IC03NiAtMjYgLTE5IC05MyAtOQotMTE1IDE4IC0xNyAyMSAtMTggNjAgLTE4IDUyNiAwIDM2MiAzIDUxMCAxMiA1MjkgMTMgMzAgNDggNTIgMzgyIDI0NiAxMzYgNzkKMjU5IDE1NSAyNzIgMTY4IDI5IDI4IDM5IDYxIDMwIDk2IC0xNSA1OCAtMzMgNTEgLTM4NiAtMTUzIC0yOTUgLTE2OSAtNDAwCi0yMzQgLTQyNSAtMjYyIC0xOSAtMjEgLTIwIC00MiAtMjUgLTUyNyAtNSAtNDg1IC02IC01MDUgLTI0IC01MTkgLTI2IC0xOQotMzAgLTE4IC02MyAxMCBsLTI4IDI0IC01IDI0MyBjLTMgMTM0IC0zIDM3NiAwIDUzOCBsNSAyOTMgMzUgMzAgYzQwIDMzIDkzMQo1NTIgOTgwIDU3MSAyMyA4IDI1IDggNDYgLTd6Ii8+CjxwYXRoIGQ9Ik0yMjE5IDIzODAgYy0zMCAtOSAtNjIgLTI2IC0yODQgLTE1OSAtODggLTUyIC0xNzUtLTEwOSAtMTkyIC0xMjcKbC0zMyAtMzIgMCAtMTQxIGMwIC04NyA0IC0xNTAgMTIgLTE2NiAxMSAtMjYgNzMgLTY1IDEwMSAtNjUgNDIgMCAzOTYgMjAyCjQzNCAyNDggMjIgMjYgMjMgMzUgMjMgMjEyIDAgMjM2IC0zIDI0NiAtNjEgMjMweiIvPgo8L2c+Cjwvc3ZnPgo=" alt="Rerun.io">
  </a>
</p>

<div align="justify">
This project provides a minimal stereo-inertial SLAM pipeline with <a href="https://github.com/ethz-asl/kalibr">Kalibr</a>-based calibration and ROS 2 VIO evaluation. The datasets, Docker environments, baseline <a href="ros2_ws/src/stereo_vio"><code>stereo_vio</code></a> package, visualization, and odometry metrics are already implemented.<br><br>

Your task is to reduce the VIO trajectory error.<br>

The system will:
- Provide stereo and stereo-inertial calibration data
- Run <a href="https://github.com/ethz-asl/kalibr">Kalibr</a> inside a ROS 1 environment
- Run the baseline stereo-inertial VIO pipeline in ROS 2
- Compare the estimated trajectory against ground truth
- Report ATE, ARE, and RPE metrics

You first calibrate the stereo-inertial system with <a href="https://github.com/ethz-asl/kalibr">Kalibr</a>, then use that calibration to improve <a href="ros2_ws/src/stereo_vio"><code>stereo_vio</code></a>. Keep the implementation clean, correct, and deterministic.
</div>

<table width="100%">
  <tr>
    <td align="center" width="50%">
      <img src="assets/vio.gif" style="width:100%; height:480px; object-fit:cover;"><br>
      <strong>VIO</strong><br>
      <sub>$$\color{blue}Blue$$ = Ground truth | $$\color{red}Red$$ = Estimated</sub>
    </td>
    <td align="center" width="50%">
      <img src="assets/sensor_stream.gif" style="width:100%; height:480px; object-fit:cover;"><br>
      <strong>Sensor Data</strong>
    </td>
  </tr>
</table>

## Dependencies
A Linux Ubuntu computer, or other equivalent.

You only need to install [Docker](https://docs.docker.com/engine/install/ubuntu/) and follow [Linux Post-Installation](https://docs.docker.com/engine/install/linux-postinstall/) to get started.

<details>
  <summary><strong>GPU Support (Optional)</strong></summary>

If your system includes an NVIDIA GPU, you can enable GPU acceleration inside Docker.

**Install:**
- [NVIDIA GPU drivers](https://www.nvidia.com/en-us/drivers/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
</details>

## Cyclone DDS tuning

The Linux kernel must be optimized to use large packet sizes. This can be done using the script provided and is done outside docker.

```shell
cd usb_dds_setup && bash install_ipfrag.sh
```

> [!NOTE]
> You need `sudo` to change this.

You can refer to further documentation [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/) or [here](https://docs.ros.org/en/jazzy/How-To-Guides/DDS-tuning.html).

## Getting Started
Clone the repo

```shell
git clone --recursive https://github.com/omniinstrument/SLAM_project.git
```

Start the Kalibr environment

```shell
bash scripts/start.sh ros1
```

Start the VIO environment

```shell
bash scripts/start.sh ros2
```

> [!NOTE]
> The start script builds the Docker images, downloads the datasets from Hugging Face, and builds the ROS workspace. Dataset download occurs only once.

> [!NOTE]
> To override the Docker base image, pass `ROS1_ROOT_IMAGE` or `ROS2_ROOT_IMAGE` when starting the environment:
> ```shell
> ROS1_ROOT_IMAGE=ubuntu:20.04 bash scripts/start.sh ros1
> ROS2_ROOT_IMAGE=ubuntu:24.04 bash scripts/start.sh ros2
> ```

## Calibration
<table width="100%">
  <tr>
    <td align="center" width="50%">
      <img src="assets/stereo.gif" style="width:100%; height:320px; object-fit:cover;"><br>
      <strong>Stereo camera calibration</strong>
    </td>
    <td align="center" width="50%">
      <img src="assets/stereo-imu.gif" style="width:100%; height:320px; object-fit:cover;"><br>
      <strong>Stereo-inertial calibration</strong>
    </td>
  </tr>
</table>

Use [Kalibr](https://github.com/ethz-asl/kalibr) in the ROS 1 container to estimate:

- Stereo camera intrinsics
- Stereo extrinsics
- Camera-IMU extrinsics
- Camera-IMU time offset, if useful

Before running [Kalibr](https://github.com/ethz-asl/kalibr), update [aprilgrid.yaml](config/aprilgrid.yaml) with the correct [target parameters](https://huggingface.co/datasets/OmniInstrument/SLAM_project). Use the `pinhole-radtan` camera model for both cameras.

The provided [imu.yaml](config/imu.yaml) file is required for stereo-inertial calibration.

Use the calibration result to update the VIO calibration consumed by [calib.yaml](ros2_ws/src/stereo_vio/config/calib.yaml).

> [!NOTE]
> Refer to the [Kalibr documentation](https://github.com/ethz-asl/kalibr/wiki) for calibration commands and configuration details.

## VIO
Run the ROS 2 VIO pipeline:

```shell
ros2 launch stereo_vio stereo_vio.launch.py
```

<table width="100%">
  <tr>
    <td align="center" width="50%">
      <img src="assets/optical_flow.gif" style="width:100%; height:480px; object-fit:cover;"><br>
      <strong>Tracking</strong><br>
      <sub>$$\color{blue}Blue$$ = Ground truth | $$\color{red}Red$$ = Estimated</sub>
    </td>
    <td align="center" width="50%">
      <img src="assets/metrics.gif" style="width:100%; height:480px; object-fit:cover;"><br>
      <strong>Trajectory metrics</strong><br>
    </td>
  </tr>
</table>

The launch file runs the VIO pipeline, visualization, and trajectory-error computation, then closes automatically shortly after bag playback finishes.

Your final VIO estimate must be published on `/stereo_vio/odom` (`nav_msgs/Odometry`) topic for evaluation.

> [!NOTE]
> The final report is saved in the [output](output) folder as `stereo_vio_odometry_error.txt`.

> [!WARNING]
> Incase you get an error like `ros2: failed to increase socket receive buffer size to at least 33554432 bytes, current is 425984 bytes` that means you Linux kernel is not tuned for large packets. Refer to [Cyclone DDS tuning](#cyclone-dds-tuning)

> [!NOTE]
> For GUI layout changes, save the Rerun blueprint to [rerun_wrapper.rbl](ros2_ws/src/rerun_wrapper/template/rerun_wrapper.rbl).

## Instructions
- You are free to improve [stereo_vio](ros2_ws/src/stereo_vio) using any classical, optimization-based, learning-based, or hybrid method.
- You are free to use external libraries.
- You are free to use AI code editors or AI agents, your code can be 100% AI generated.
- You are free to use any programming language, including Python, C++, Rust, and/or CUDA.
- **You are not allowed to modify the odometry error computation or report generation in [odometry_error_node.hpp](ros2_ws/src/stereo_vio/include/stereo_vio/odometry_error_node.hpp) or [odometry_error_node.cpp](ros2_ws/src/stereo_vio/src/odometry_error_node.cpp).**

### Submission
Upload the [output](output) directory and share it with us.

Also include a brief summary of:
- The calibration files used
- The [stereo_vio](ros2_ws/src/stereo_vio) changes made

## Bugs/Support
If you come across any bugs or have questions, feel free to open an Issue or reach out to [Jagennath Hari](mailto:hari@omniinstrument.com).

## License
This software and dataset are released under the [MIT License](LICENSE).

## Acknowledgment
This work integrates several powerful research papers, libraries, and open-source tools:

- [**Kalibr**](https://github.com/ethz-asl/kalibr)
- [**GTSAM**](https://gtsam.org/)
- [**Rerun**](https://rerun.io/)

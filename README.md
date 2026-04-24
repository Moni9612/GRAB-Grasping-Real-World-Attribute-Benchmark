# GRAB:Grasping-Real-World-Attribute-Benchmark

GRAB is a comprehensive benchmark designed to evaluate grasping-in-clutter performance in real-world food waste contaminant sorting. It incorporates:
(1) diverse, novel domain-specific deformable datasets,
(2) advanced 6D grasp pose estimation, and
(3) explicit evaluation of pre-grasp conditions through graspability metrics.

![Methodology](https://github.com/user-attachments/assets/952ebc4e-c944-45aa-98cb-6a4d267287da)

This repository provides setup instructions for two grasping benchmark pipelines: a parallel grasping pipeline for rigid and adaptive force-controlled parallel grippers, and a suction-based grasping pipeline.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10.12
- CUDA toolkit version 12.1
- PyTorch 2.1.0

## Installation

### Set up UR10 Robot Arm.

Install Universal Robots ROS 2 Driver: [GitHub Repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

### Set up Moveit2

Install MoveIt 2 version: 2.5.9 (Humble compatible)

### Set up RealSense Camera

- Refer to the official [RealSense ROS2 Installation Guide](https://github.com/realsenseai/realsense-ros/blob/ros2-development/README.md#installation-on-ubuntu)

  - Navigate to **Installation on Ubuntu**
  - Follow **Step 2: Install the latest Intel® RealSense™ SDK 2.0** (use Option 2)
  - Then complete **Step 3: Install the ROS Wrapper for Intel® RealSense™ Cameras**


## Parallel Grasping Pipeline.

- Install **AnyGrasp SDK** inside a Docker container, as it supports multiple CUDA and Python versions:  
  [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk)

- Clone the `anygrasp_benchmark_ws` repository using the following command:
  ```bash
  git clone --recurse-submodules https://github.com/Moni9612/GRAB-Grasping-Real-World-Attribute-Benchmark.git

- Install **SuctionNet**, including the following components:  
  - [SuctionNetAPI](https://github.com/graspnet/suctionnetAPI)  
  - [suctionnet-baseline](https://github.com/graspnet/suctionnet-baseline)


### Clone Required Repositories

Clone the following repositories into your home directory:

- [anygrasp_benchmark](https://github.com/Moni9612/anygrasp_benchmark_ws.git)


Stay tuned - the GRAB stack is under updates.

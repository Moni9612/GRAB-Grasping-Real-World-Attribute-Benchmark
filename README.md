# GRAB:Grasping-Real-World-Attribute-Benchmark

GRAB is a comprehensive benchmark designed to evaluate grasping-in-clutter performance in real-world food waste contaminant sorting. It incorporates:
(1) diverse, novel domain-specific deformable datasets,
(2) advanced 6D grasp pose estimation, and
(3) explicit evaluation of pre-grasp conditions through graspability metrics.

![Methodology](https://github.com/user-attachments/assets/952ebc4e-c944-45aa-98cb-6a4d267287da)

Stay tuned - the GRAB stack will be available soon.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Universal Robots ROS 2 Driver: [GitHub Repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- Python 3.10.12
- CUDA toolkit version 12.1
- PyTorch 2.1.0
- MoveIt 2 version: 2.5.9 (Humble compatible)

## Installation

1. Install below 6D grasp pose detection algorithms:

    - Install Anygrasp_sdk inside a docker container since it supports different CUDA and python versions: [Anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk)

    - Install Suctionnet ([SuctionNetAPI](https://github.com/graspnet/suctionnetAPI) and [suctionnet-baseline](https://github.com/graspnet/suctionnet-baseline))

2. Install Intel® RealSense™ SDK 2.0 since we are using a D415 realsense camera

    - Go to this link [Realsense SDK](https://github.com/realsenseai/realsense-ros/blob/ros2-development/README.md#installation-on-ubuntu)
    - Then go to installation on ubuntu.
    - Go to Step 2: Install latest Intel® RealSense™ SDK 2.0  and follow option 2
    - Then Step 3: Install ROS Wrapper for Intel® RealSense™ cameras Step

3. Then clone below github repositories into your home location:

    - anygrasp_benchmark
    - Suctionnet_move_robot
    - relay_contol_ws
      

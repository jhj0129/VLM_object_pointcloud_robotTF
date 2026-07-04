# VLM object pointcloud robotTF

This repository contains a staged ROS2 pipeline for VLM-guided object perception and robot-frame alignment.

## Current status

Stage 1 is completed.

Stage 1 implements a camera-only scan-once pipeline:

- RealSense RGB-D input at 424x240 15 FPS
- C++ ROS2 scan-once node
- 20-frame depth median filtering
- object PointCloud2 generation
- grasp_point and push_point publication
- RViz visualization in camera_color_optical_frame

## Stage plan

Stage 1:
Camera-only object pointcloud and grasp/push point in camera frame.

Stage 2:
Transform object information into the robot frame using calibrated camera extrinsics.

Stage 3:
Integrate Qwen/VLM command grounding and execute real-world object interaction through MoveIt.

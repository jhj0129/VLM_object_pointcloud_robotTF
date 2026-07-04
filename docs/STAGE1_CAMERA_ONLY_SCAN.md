# Stage 1 - Camera-only scan-once object pointcloud

## Goal

This stage verifies the camera-only perception pipeline without robot TF, MoveIt, or real robot motion.

The system scans a static object once using RealSense RGB-D frames, computes a stable object pointcloud, and publishes grasp/push candidate points in the camera optical frame.

## Coordinate frame

All outputs are published in:

camera_color_optical_frame

No robot frame is used in Stage 1.

## Input topics

/test_rs/color/image_raw
/test_rs/depth/image_raw
/test_rs/color/camera_info

The RealSense pipeline uses 424x240 RGB-D at 15 FPS.

## Output topics

/vlm_camera_scan/object_cloud
/vlm_camera_scan/grasp_point
/vlm_camera_scan/push_point
/vlm_camera_scan/object_markers
/vlm_camera_scan/annotated_image
/vlm_camera_scan/mask_image

## Data rate

Input RGB:

424 x 240 x 3 bytes x 15 FPS = about 4.58 MB/s

Input depth:

424 x 240 x 2 bytes x 15 FPS = about 3.05 MB/s

Total RGB-D input:

about 7.63 MB/s

Scan-once capture:

20 RGB frames   = about 6.1 MB
20 depth frames = about 4.1 MB
Total           = about 10.2 MB

Result output is lightweight:

object_cloud, 5000 points = about 60 KB
annotated image           = about 305 KB
mask image                = about 102 KB

## Run

Terminal 1 - RealSense RGB-D camera

Keep this terminal running.

cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_only_ee_joystick_control rs_py_rgbd_camera.launch.py

Terminal 2 - C++ scan-once node

Keep this terminal running after the scan, because it republishes the result for RViz.

cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_only_ee_joystick_control camera_only_scan_once_cpp.launch.py

Terminal 3 - RViz

source /opt/ros/humble/setup.bash
rviz2

RViz settings:

Fixed Frame: camera_color_optical_frame
PointCloud2: /vlm_camera_scan/object_cloud
MarkerArray: /vlm_camera_scan/object_markers

## Success criteria

1. RealSense RGB-D is stable at about 15 FPS.
2. The C++ scan-once node captures 20 frames.
3. The node computes a depth-median object pointcloud.
4. RViz shows object_cloud as a set of points.
5. grasp_point and push_point are published.
6. All outputs use camera_color_optical_frame.
7. No ARM_BASE_LINK, robot TF, or MoveIt is used in this stage.

## Optical frame note

The object may look tilted or rotated in RViz because the optical frame uses the camera convention:

+X: image right
+Y: image down
+Z: camera forward / depth direction

Robot-frame alignment will be handled in Stage 2.

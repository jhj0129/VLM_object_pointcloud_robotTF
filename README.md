# VLM 기반 빨간 물체 인식 → MoveIt → 실제 DROK 로봇팔 Grasp 성공 기록

## 성공 요약

RealSense RGB-D 카메라로 빨간 물체를 검출하고, depth 정보를 이용해 3D grasp point를 만든 뒤, 해당 좌표를 `ARM_BASE_LINK` 기준으로 TF 변환하여 MoveIt 기반 실제 DROK 로봇팔을 이동시키는 데 성공했다.

최종적으로 다음 흐름이 실제 로봇에서 동작했다.

```text
RealSense RGB-D
→ 빨간색 물체 mask / depth 기반 3D point 생성
→ camera_color_optical_frame 기준 grasp point 생성
→ ARM_BASE_LINK 기준으로 TF 변환
→ /ee_grasp/target_point로 전달
→ MoveIt grasp planner 실행
→ moveit_to_rmd_bridge.py를 통해 실제 모터 명령 전송
→ JOINT7 그리퍼 close 명령으로 물체 grasp
```

---

## 핵심 성공 포인트

### 1. 부팅 후 실제 로봇 HOME 정렬이 필요함

DROK 로봇팔은 부팅할 때마다 모터 raw 기준이 미세하게 달라질 수 있다. 따라서 MoveIt/RViz를 먼저 실행하면 실제 로봇 자세와 RViz 모델 자세가 어긋나거나, 스폰 직후 링크가 서로 겹쳐 보이는 문제가 발생할 수 있다.

이번 성공에서는 MoveIt을 켜기 전에 실제 로봇을 저장된 HOME 위치로 먼저 이동시켰고, 그 뒤 `joystick_node_92`로 `/joint_states`를 publish한 상태에서 MoveIt/RViz를 실행했다. 이 순서로 실행하니 RViz에서 보이는 로봇 형상이 실제 로봇 형상과 어긋나지 않았다.

성공한 HOME 동기화 루틴은 다음과 같다.

```text
실제 로봇을 HOME 자세로 맞춤
→ capture_arm_session_home.py로 현재 모터 raw HOME 캡처
→ moveit_to_rmd_bridge.py 실행
→ /arm_controller/follow_joint_trajectory action으로 저장된 HOME joint 값을 실제 모터에 전송
→ joystick_node_92 실행
→ /joint_states publish
→ MoveIt/RViz 실행
```

중요한 점은 `joystick_node_92`와 `moveit_to_rmd_bridge.py`가 반드시 같은 `/tmp/drok_session_home.yaml`을 사용해야 한다는 것이다.

---

## HOME 캡처 명령

실제 로봇을 물리적으로 HOME 자세에 맞춘 뒤 실행한다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/capture_arm_session_home.py \
  --base src/arm_control/config/real_home.yaml \
  --out /tmp/drok_session_home.yaml \
  --samples 5
```

`/tmp/drok_session_home.yaml`은 부팅 세션마다 달라질 수 있는 임시 캘리브레이션 파일이므로 Git에 올리지 않는다.

---

## 실제 로봇을 저장 HOME으로 이동

MoveIt/RViz를 켜기 전에 bridge를 실행하고, 아래 action 명령으로 실제 로봇을 저장된 HOME 자세로 먼저 이동시킨다.

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6], points: [{positions: [0.0, 0.385378389294, -0.377267456410, 0.0, 0.0, -0.782896224400], time_from_start: {sec: 8, nanosec: 0}}]}}"
```

이 단계를 생략하면 RViz에서 로봇 링크가 겹쳐 보이거나 실제 로봇 자세와 MoveIt 모델 자세가 다르게 보일 수 있다.

---

## 실제 실행에 사용한 주요 노드와 토픽

### 로봇 관절 상태

```text
joystick_node_92
→ /joint_states
→ MoveIt / RViz / ee_grasp_pose_plan_node
```

최종 성공 루틴에서는 `/joint_states_raw`와 `joint_state_keepalive.py`를 사용하지 않았다.

### MoveIt 실행 bridge

```text
MoveIt trajectory
→ /arm_controller/follow_joint_trajectory
→ moveit_to_rmd_bridge.py
→ 실제 RMD 모터 명령
```

### 그리퍼 제어

```text
/gripper_controller/gripper_cmd
→ moveit_to_rmd_bridge.py
→ JOINT7 기반 그리퍼 open/close
```

### 카메라 및 VLM grasp point

```text
RealSense RGB-D
→ /test_rs/color/image_raw
→ /test_rs/aligned_depth_to_color/image_raw
→ camera_only_scan_once_cpp_node
→ /vlm_camera_scan/grasp_point
→ camera_to_robot_tf_cpp_node
→ /vlm_robot_tf/grasp_point
→ relay
→ /ee_grasp/target_point
→ ee_grasp_pose_plan_node
```

---

## 최종 실행 순서

### Terminal 1: 실제 관절 상태 publisher

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control joystick_node_92 \
  --ros-args \
  --params-file /tmp/drok_session_home.yaml
```

### Terminal 2: 실제 로봇 bridge

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control moveit_to_rmd_bridge.py \
  --ros-args \
  --params-file /tmp/drok_session_home.yaml \
  -p dry_run:=false \
  -p default_max_speed:=3 \
  -p gripper_max_speed:=100
```

### Terminal 3: MoveIt/RViz

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch arm_only_moveit_config real_moveit.launch.py
```

### Terminal 4: 카메라 + TF + 좌표변환

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x -0.020 \
  --y 0.290 \
  --z 0.450 \
  --qx 0.653281482438 \
  --qy -0.653281482438 \
  --qz 0.270598050073 \
  --qw -0.270598050073 \
  --frame-id ARM_BASE_LINK \
  --child-frame-id camera_color_optical_frame &

ros2 run vlm_object_pointcloud_robot_tf camera_to_robot_tf_cpp_node &

ros2 run vlm_object_pointcloud_robot_tf rs_py_rgbd_camera_node \
  --ros-args \
  -p width:=640 \
  -p height:=480 \
  -p fps:=15 \
  -p color_frame_id:=camera_color_optical_frame \
  -p depth_frame_id:=camera_color_optical_frame \
  -p color_topic:=/test_rs/color/image_raw \
  -p color_info_topic:=/test_rs/color/camera_info \
  -p depth_topic:=/test_rs/depth/image_raw \
  -p depth_info_topic:=/test_rs/depth/camera_info \
  -p aligned_depth_topic:=/test_rs/aligned_depth_to_color/image_raw \
  -p aligned_depth_info_topic:=/test_rs/aligned_depth_to_color/camera_info
```

### Terminal 5: grasp planner 실제 실행

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch arm_only_ee_joystick_control ee_grasp_pose_plan.launch.py \
  execute_plan:=true
```

성공 시 다음 로그가 출력된다.

```text
This node will use its own /joint_states cache for grasp IK start state.
execute_plan   = true
```

### Terminal 6: 빨간 물체 scan 후 target 전달

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vlm_object_pointcloud_robot_tf camera_only_scan_once_cpp_node \
  --ros-args \
  -p rgb_topic:=/test_rs/color/image_raw \
  -p depth_topic:=/test_rs/aligned_depth_to_color/image_raw \
  -p info_topic:=/test_rs/color/camera_info \
  -p frame_id:=camera_color_optical_frame \
  -p target_color:=red \
  -p capture_frames:=30 \
  -p depth_scale:=0.001 \
  -p raw_depth_min:=0.30 \
  -p raw_depth_max:=1.50 \
  -p min_mask_pixels:=150 \
  -p min_valid_depth_pixels:=100 \
  -p max_cloud_points:=8000
```

이후 `/vlm_robot_tf/grasp_point`를 `/ee_grasp/target_point`로 relay하여 grasp planner에 target을 전달한다.

---

## Gripper 명령

팔이 물체 근처에 도착한 뒤 JOINT7 기반 gripper command를 사용한다.

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 18.880, max_effort: 0.0}}"
```

더 조일 때:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 25.400, max_effort: 0.0}}"
```

더 조일 때:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 31.930, max_effort: 0.0}}"
```

---

## 주의사항

- `/tmp/drok_session_home.yaml`은 Git에 올리지 않는다.
- 로봇을 켤 때마다 실제 HOME 캡처를 다시 해야 할 수 있다.
- `joystick_node_92`와 `moveit_to_rmd_bridge.py`는 반드시 같은 home yaml을 사용해야 한다.
- MoveIt/RViz는 `/joint_states`가 먼저 publish되고 실제 로봇이 HOME 위치로 이동한 뒤 실행하는 것이 안정적이다.
- 최종 성공 루틴에서는 `/joint_states_raw`와 `joint_state_keepalive.py`를 사용하지 않는다.
- `execute_plan:=true`가 실제 노드에서 `execute_plan = true`로 출력되는지 확인해야 한다.

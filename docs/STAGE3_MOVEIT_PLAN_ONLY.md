# Stage 3 MoveIt Plan-Only 정리

## 현재 성공한 내용

현재 VLM/카메라 기반 목표점을 MoveIt 계획 경로로 연결하는 단계까지 성공했다.

성공한 흐름은 다음과 같다.

1. RealSense RGB-D 카메라에서 물체를 scan-once 방식으로 추출
2. 카메라 좌표계 `camera_color_optical_frame`에서 물체 point cloud 생성
3. 물체 point cloud와 grasp/push point를 로봇 기준 좌표계 `ARM_BASE_LINK`로 변환
4. MoveIt RViz에서 로봇 모델과 물체 point cloud/marker를 함께 표시
5. `/vlm_robot_tf/grasp_point`를 기반으로 MoveIt용 pose 생성
   - `/vlm_moveit/pregrasp_pose`
   - `/vlm_moveit/grasp_target_pose`
   - `/vlm_moveit/target_markers`
6. MoveIt plan-only 성공
   - 현재 자세 -> pregrasp
   - 현재 자세 -> grasp target
   - 현재 자세 -> pregrasp -> grasp target 2구간 sequence

아직 실제 로봇 execute는 하지 않았다.

---

## 최종 성공 로그 요약

예시 목표 pose는 다음과 같았다.

```text
pregrasp_pose:
  frame: ARM_BASE_LINK
  position: x=0.606, y=0.182, z=0.399

grasp_target_pose:
  frame: ARM_BASE_LINK
  position: x=0.606, y=0.182, z=0.299
```

MoveIt sequence planning 결과:

```text
SEQUENCE PLAN 1 SUCCESS: current -> pregrasp
SEQUENCE PLAN 2 SUCCESS: pregrasp -> grasp
SEQUENCE PLAN SUCCESS: published plan1 + plan2 to /display_planned_path
```

중요 설정:

```text
end_effector_link = gripper_tcp
use_position_only = true
execute = 사용하지 않음
```

처음에는 `LINK6`를 목표 링크로 사용했을 때 planning 실패가 발생했다. 이후 실제 그리퍼 끝점인 `gripper_tcp`를 목표 링크로 사용하자 planning이 성공했다.

---

## 주요 토픽

로봇 좌표계 기준 물체 정보:

```text
/vlm_robot_tf/object_cloud
/vlm_robot_tf/grasp_point
/vlm_robot_tf/push_point
/vlm_robot_tf/object_markers
```

MoveIt 목표 pose:

```text
/vlm_moveit/pregrasp_pose
/vlm_moveit/grasp_target_pose
/vlm_moveit/target_markers
```

MoveIt planned path:

```text
/display_planned_path
```

---

## 실행 명령

### Terminal 1 - Camera

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash
ros2 launch vlm_object_pointcloud_robot_tf rs_py_rgbd_camera.launch.py
```

### Terminal 2 - Stage 1 scan-once

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash
ros2 launch vlm_object_pointcloud_robot_tf camera_only_scan_once_cpp.launch.py
```

### Terminal 3 - MoveIt + object overlay

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash
ros2 launch vlm_object_pointcloud_robot_tf stage2_moveit_object_overlay.launch.py
```

### Terminal 4 - target pose preview

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash
ros2 launch vlm_object_pointcloud_robot_tf robot_target_pose_preview.launch.py \
  target_offset_z:=0.08 \
  pregrasp_extra_z:=0.10
```

### Terminal 5 - sequence plan-only

```bash
cd ~/VLM_object_pointcloud_robotTF
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash
ros2 launch vlm_object_pointcloud_robot_tf moveit_plan_sequence_pregrasp_to_grasp.launch.py
```

---

## RViz Add 항목

Global Options:

```text
Fixed Frame: ARM_BASE_LINK
```

필수:

```text
RobotModel
MotionPlanning
TF
```

물체 점군:

```text
Add -> PointCloud2
Topic: /vlm_robot_tf/object_cloud
Size: 0.005
Style: Flat Squares
```

물체 marker:

```text
Add -> MarkerArray
Topic: /vlm_robot_tf/object_markers
```

target/pregrasp marker:

```text
Add -> MarkerArray
Topic: /vlm_moveit/target_markers
```

의미:

```text
하늘색 구 = pregrasp_pose
빨간 구 = grasp_target_pose
파란 화살표 = pregrasp -> grasp 접근 방향
```

선택:

```text
Add -> Pose
Topic: /vlm_moveit/pregrasp_pose

Add -> Pose
Topic: /vlm_moveit/grasp_target_pose
```

Pose 화살표가 너무 크면 다음 값으로 줄인다.

```text
Shaft Length: 0.08
Shaft Radius: 0.01
Head Length: 0.03
Head Radius: 0.02
```

---

## 현재 한계

현재는 `use_position_only=true`이다.

즉 `gripper_tcp`의 위치만 맞추고, 그리퍼 방향은 강제하지 않는다. 실제 로봇 실행에는 아직 적합하지 않다.

실제 실행 전에는 반드시 다음 문제를 해결해야 한다.

1. 배선 보호용 joint 회전량 검사
2. JOINT6 한 바퀴 이상 회전 방지
3. gripper_tcp의 full pose orientation 고정
4. 위/아래 grasp가 아니라 양옆 side grasp 생성
5. 실제 execute 전 safety gate 추가

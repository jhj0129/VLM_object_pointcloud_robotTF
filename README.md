# VLM Object PointCloud RobotTF + DROK Arm Side Grasp Pipeline

이 문서는 ROS 2 Humble, RealSense D455, MoveIt2, RViz, DROK 로봇팔을 이용해
카메라에서 얻은 물체 위치를 로봇 좌표계로 변환하고, MoveIt에서 side grasp 방식으로
pregrasp -> grasp 계획을 생성하는 현재 개발 단계의 README입니다.

현재 단계는 실제 로봇 실행이 아니라 plan-only 검증 단계입니다.
RViz에서 side grasp 궤적이 정상적으로 생성되는 것까지 확인했습니다.

---

## 1. 현재 성공한 기능

현재까지 완료된 기능은 다음과 같습니다.

- RealSense RGB-D 카메라 실행
- 카메라 기반 object pointcloud 생성
- grasp point를 ARM_BASE_LINK 기준 좌표로 변환
- RViz에서 object cloud, marker, TF 표시
- side grasp target pose 생성
- pregrasp pose 생성
- pregrasp = grasp - 0.10 m along X
- gripper_tcp 방향 quaternion 적용
- current -> pregrasp full pose planning
- pregrasp -> grasp Cartesian straight planning
- /display_planned_path로 RViz plan-only 궤적 표시
- JOINT6 continuous -> revolute 수정
- JOINT4, JOINT5, JOINT6 배선 보호 limit 반영
- position_only_ik: false 설정
- 실제 로봇 실행 전 안전 보정 완료

---

## 2. 전체 시스템 구조

전체 흐름은 다음과 같습니다.

```text
RealSense D455
  -> RGB-D image / depth / pointcloud
  -> object point extraction
  -> camera frame point
  -> ARM_BASE_LINK frame point
  -> grasp pose
  -> pregrasp pose
  -> MoveIt plan-only side grasp
  -> RViz DisplayTrajectory
```

side grasp 방식은 다음과 같습니다.

```text
pregrasp ---- +X straight approach ----> grasp
```

현재 설정에서는 grasp와 pregrasp의 orientation은 동일하게 유지합니다.

---

## 3. 주요 좌표계와 토픽

### Fixed Frame

RViz Fixed Frame:

```text
ARM_BASE_LINK
```

### 주요 토픽

```text
/vlm_robot_tf/grasp_point
/vlm_robot_tf/object_cloud
/vlm_robot_tf/object_markers
/vlm_moveit/grasp_target_pose
/vlm_moveit/pregrasp_pose
/vlm_moveit/target_markers
/display_planned_path
```

### 주요 링크

```text
ARM_BASE_LINK
gripper_tcp
```

---

## 4. 현재 side grasp pose 설정

현재 테스트에 성공한 side grasp 설정입니다.

```text
side_grasp_mode: true

target_offset_x: 0.0
target_offset_y: 0.0
target_offset_z: 0.0

pregrasp_offset_x: -0.10
pregrasp_offset_y: 0.0
pregrasp_offset_z: 0.0
```

따라서 pose 관계는 다음과 같습니다.

```text
grasp.x - pregrasp.x = 0.100 m
grasp.y = pregrasp.y
grasp.z = pregrasp.z
grasp.orientation = pregrasp.orientation
```

현재 테스트한 gripper_tcp quaternion은 다음과 같습니다.

```text
target_qx: 1.000
target_qy: 0.000
target_qz: -0.004
target_qw: -0.002
```

이는 `tf2_echo ARM_BASE_LINK gripper_tcp`에서 측정한 현재 side grasp 방향입니다.

---

## 5. DROK Arm 보정 내용

DROK_ARM_EEcontrol 저장소에서 side grasp를 위해 적용한 주요 보정입니다.

### JOINT1 Home 보정

```text
JOINT1 raw home: 5.109444444444445 deg
```

### JOINT6 Home 보정

```text
JOINT6 raw home: -20.668333333333333 deg
MoveIt JOINT6 home: -0.78289622440 rad
```

### 배선 보호 limit

```text
JOINT4 lower: -5.89 rad
JOINT4 upper:  2.64 rad

JOINT5 lower: -1.52 rad
JOINT5 upper:  1.32 rad

JOINT6 lower: -2.87729 rad
JOINT6 upper:  1.31150 rad
```

### JOINT6 URDF 수정

JOINT6는 기존 continuous에서 revolute로 수정했습니다.

```xml
<joint name="JOINT6" type="revolute">
  <limit lower="-2.87729" upper="1.31150" effort="35" velocity="1.0"/>
</joint>
```

### IK 설정

side grasp는 방향이 중요하기 때문에 position-only IK를 껐습니다.

```yaml
position_only_ik: false
```

---

## 6. 실행 방법

아래 명령은 현재 성공한 side grasp plan-only 테스트 순서입니다.

---

### Terminal 0 - 기존 노드 정리와 모델 확인

```bash
pkill -f robot_target_pose_cpp_node
pkill -f moveit_plan_side_grasp_cpp_node

cd ~/DROK_ARM_EEcontrol

source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p /tmp/drok_urdf_check

xacro install/arm_only_moveit_config/share/arm_only_moveit_config/config/arm_only_gripper.urdf.xacro   > /tmp/drok_urdf_check/expanded_arm_only_gripper_after.urdf

grep -n 'name="JOINT6"' -A8 -B3 /tmp/drok_urdf_check/expanded_arm_only_gripper_after.urdf

grep -n 'position_only_ik' install/arm_only_moveit_config/share/arm_only_moveit_config/config/kinematics.yaml
```

정상 확인값:

```text
JOINT6 type="revolute"
lower="-2.87729"
upper="1.31150"
position_only_ik: false
```

---

### Terminal 1 - RealSense 실행

```bash
cd ~/VLM_object_pointcloud_robotTF

source /opt/ros/humble/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

ros2 launch vlm_object_pointcloud_robot_tf rs_py_rgbd_camera.launch.py
```

---

### Terminal 2 - Camera scan once 실행

```bash
cd ~/VLM_object_pointcloud_robotTF

source /opt/ros/humble/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

ros2 launch vlm_object_pointcloud_robot_tf camera_only_scan_once_cpp.launch.py
```

---

### Terminal 3 - MoveIt + RViz overlay 실행

```bash
cd ~/VLM_object_pointcloud_robotTF

source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

ros2 launch vlm_object_pointcloud_robot_tf stage2_moveit_object_overlay.launch.py
```

RViz 설정:

```text
Fixed Frame: ARM_BASE_LINK
```

RViz Add 항목:

```text
RobotModel
TF
MotionPlanning
PointCloud2      /vlm_robot_tf/object_cloud
MarkerArray      /vlm_robot_tf/object_markers
MarkerArray      /vlm_moveit/target_markers
Pose             /vlm_moveit/pregrasp_pose
Pose             /vlm_moveit/grasp_target_pose
DisplayTrajectory /display_planned_path
```

---

### Terminal 4 - side grasp target pose 생성

```bash
cd ~/VLM_object_pointcloud_robotTF

source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

ros2 launch vlm_object_pointcloud_robot_tf robot_target_pose_preview.launch.py   side_grasp_mode:=true   target_offset_x:=0.0   target_offset_y:=0.0   target_offset_z:=0.0   pregrasp_offset_x:=-0.10   pregrasp_offset_y:=0.0   pregrasp_offset_z:=0.0   target_qx:=1.000   target_qy:=0.000   target_qz:=-0.004   target_qw:=-0.002
```

정상 로그 예시:

```text
SIDE GRASP target: frame=ARM_BASE_LINK, pos=(...), quat=(1.0000 0.0000 -0.0040 -0.0020)
SIDE GRASP pregrasp: pos=(...), offset=(-0.100, 0.000, 0.000)
```

---

### Terminal 5 - pose topic 확인

```bash
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

echo "===== grasp ====="
ros2 topic echo /vlm_moveit/grasp_target_pose --once

echo "===== pregrasp ====="
ros2 topic echo /vlm_moveit/pregrasp_pose --once
```

확인할 것:

```text
grasp.x - pregrasp.x = 0.100
grasp.y = pregrasp.y
grasp.z = pregrasp.z

orientation.x ~= 1.000
orientation.y ~= 0.000
orientation.z ~= -0.004
orientation.w ~= -0.002
```

---

### Terminal 6 - side grasp plan-only 실행

```bash
cd ~/VLM_object_pointcloud_robotTF

source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash
source ~/VLM_object_pointcloud_robotTF/install/setup.bash

ros2 launch vlm_object_pointcloud_robot_tf moveit_plan_side_grasp.launch.py
```

성공 로그 예시:

```text
SIDE GRASP VECTOR pregrasp -> grasp = (0.100, 0.000, 0.000)
SIDE GRASP PLAN 1 SUCCESS
SIDE GRASP CARTESIAN SUCCESS
SIDE GRASP PLAN ONLY SUCCESS
```

---

### Terminal 7 - gripper_tcp quaternion 확인용

필수는 아닙니다. gripper_tcp 방향을 다시 확인할 때 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash

ros2 run tf2_ros tf2_echo ARM_BASE_LINK gripper_tcp
```

확인할 부분:

```text
Rotation: in Quaternion (xyzw)
```

현재 테스트한 값:

```text
xyzw = [1.000, 0.000, -0.004, -0.002]
```

---

## 7. 주요 노드

### VLM_object_pointcloud_robotTF

```text
camera_only_scan_once_cpp_node
camera_to_robot_tf_cpp_node
robot_target_pose_cpp_node
moveit_plan_to_pose_cpp_node
moveit_plan_sequence_cpp_node
moveit_plan_side_grasp_cpp_node
```

### Side grasp plan node

```text
moveit_plan_side_grasp_cpp_node
```

동작:

```text
1. current state -> pregrasp
   - full pose plan

2. pregrasp -> grasp
   - Cartesian straight approach

3. execute 호출 없음
   - plan-only
   - /display_planned_path publish
```

---

## 8. 현재 GitHub 커밋 상태

현재 성공 지점은 GitHub에 다음 커밋으로 저장했습니다.

### DROK_ARM_EEcontrol

```text
stage4: calibrate real robot for side grasp
```

포함 내용:

```text
- 실제 로봇 Home 보정
- JOINT1 raw home 보정
- JOINT6 raw home 보정
- JOINT4, JOINT5, JOINT6 배선 보호 limit
- JOINT6 continuous -> revolute
- position_only_ik false
- side grasp용 gripper 방향 보정
```

### VLM_object_pointcloud_robotTF

```text
stage4: add VLM side grasp plan-only pipeline
```

포함 내용:

```text
- side grasp target pose 생성
- pregrasp pose 생성
- RViz marker 생성
- target quaternion 입력 가능
- moveit_plan_side_grasp_cpp_node 추가
- current -> pregrasp full pose plan
- pregrasp -> grasp Cartesian plan
```

---

## 9. Troubleshooting

### 1. quat이 identity로 나오는 경우

문제 로그:

```text
quat=(0.0000 0.0000 0.0000 1.0000)
```

해결:

```bash
target_qx:=1.000
target_qy:=0.000
target_qz:=-0.004
target_qw:=-0.002
```

값을 `robot_target_pose_preview.launch.py` 실행 인자에 넣어야 합니다.

---

### 2. JOINT6 continuous 에러

문제 로그:

```text
Cannot specify position limits for continuous joint JOINT6
```

해결:

확장 URDF에서 JOINT6가 revolute인지 확인합니다.

```bash
xacro install/arm_only_moveit_config/share/arm_only_moveit_config/config/arm_only_gripper.urdf.xacro   > /tmp/drok_urdf_check/expanded_arm_only_gripper_after.urdf

grep -n 'name="JOINT6"' -A8 -B3 /tmp/drok_urdf_check/expanded_arm_only_gripper_after.urdf
```

정상:

```text
JOINT6 type="revolute"
lower="-2.87729"
upper="1.31150"
```

---

### 3. Using position only ik 로그가 뜨는 경우

문제 로그:

```text
Using position only ik
```

해결:

```bash
grep -n 'position_only_ik' ~/DROK_ARM_EEcontrol/install/arm_only_moveit_config/share/arm_only_moveit_config/config/kinematics.yaml
```

정상:

```text
position_only_ik: false
```

---

### 4. PLAN 1 실패

문제 로그:

```text
SIDE GRASP PLAN 1 FAILED
```

가능 원인:

```text
- target quaternion이 실제 gripper_tcp 방향과 다름
- pregrasp 위치가 workspace 끝에 가까움
- 목표 z가 너무 낮음
- collision 때문에 접근이 막힘
```

완화 테스트:

```bash
ros2 launch vlm_object_pointcloud_robot_tf robot_target_pose_preview.launch.py   side_grasp_mode:=true   target_offset_x:=-0.05   target_offset_y:=0.0   target_offset_z:=0.03   pregrasp_offset_x:=-0.08   target_qx:=1.000   target_qy:=0.000   target_qz:=-0.004   target_qw:=-0.002
```

---

### 5. Cartesian 실패

문제 로그:

```text
SIDE GRASP CARTESIAN FAILED
```

가능 원인:

```text
- pregrasp -> grasp 직선 이동 중 IK가 끊김
- collision 발생
- 접근 거리가 너무 김
- 목표 자세가 관절 한계 근처
```

완화 방향:

```text
- pregrasp_offset_x를 -0.10에서 -0.08로 줄이기
- target_offset_z를 0.02 또는 0.03으로 올리기
- target_offset_x를 -0.03 또는 -0.05로 줄이기
```

---

## 10. 아직 하지 않은 것

현재는 plan-only 단계입니다.

아직 하지 않은 것:

```text
- 실제 로봇 execute
- gripper close 동작 연동
- grasp 후 lift motion
- place motion
- 자연어 VLM object selection
- mask 기반 object segmentation 자동화
- collision object 자동 삽입
- 실패 시 자동 재계획
```

---

## 11. 다음 개발 목표

다음 단계 목표:

```text
1. plan-only side grasp를 실제 로봇 execute로 확장
2. execute 전 hard safety gate 추가
3. JOINT4, JOINT5, JOINT6 limit runtime 검사
4. gripper close command 연동
5. grasp 후 lift motion 추가
6. VLM 자연어 object selection 연결
7. segmentation mask 기반 pointcloud crop 자동화
8. MoveIt PlanningScene collision object 자동 삽입
```

---

## 12. 안전 주의

실제 로봇 execute 전 반드시 확인해야 합니다.

```text
- JOINT4, JOINT5, JOINT6 배선 여유
- JOINT6 raw home과 MoveIt home 동기화
- gripper_tcp 방향
- RViz plan trajectory
- 실제 로봇 주변 장애물
- emergency stop 가능 상태
```

현재 README 기준 상태는 다음입니다.

```text
Status: side grasp plan-only verified.
Real robot execution: not enabled yet.
```

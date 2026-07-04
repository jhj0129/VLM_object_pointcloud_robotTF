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

<!-- STAGE3_STATUS_START -->
## 현재 상태 - Stage 3 MoveIt Plan-Only

Stage 3 plan-only preview가 성공했다.

완료된 것:

```text
Stage 1: camera-only scan-once object point cloud
Stage 2: camera frame -> ARM_BASE_LINK robot-frame transform
Stage 2 overlay: MoveIt RViz + object cloud/markers
Stage 2-3: grasp_point -> pregrasp/grasp PoseStamped
Stage 3-1: current -> pregrasp plan-only
Stage 3-2A: current -> grasp target plan-only
Stage 3-2B: current -> pregrasp -> grasp target sequence plan-only
```

중요 설정:

```text
end_effector_link = gripper_tcp
use_position_only = true
execute = 아직 사용하지 않음
```

실제 로봇 실행은 아직 금지한다.

다음 필수 작업:

```text
1. 배선 보호용 joint 회전량 검사
2. JOINT6 winding 제한
3. gripper_tcp full pose orientation 고정
4. 위/아래 grasp가 아닌 양옆 side grasp 생성
5. pregrasp를 z-offset 방식이 아니라 수평 접근 방식으로 변경
6. pregrasp -> grasp Cartesian path
7. execute_enabled safety gate
```

자세한 내용:

```text
docs/STAGE3_MOVEIT_PLAN_ONLY.md
docs/NEXT_STEPS_EXECUTION_SAFETY.md
```

<!-- STAGE3_STATUS_END -->

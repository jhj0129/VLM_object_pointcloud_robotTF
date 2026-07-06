# Stage 5 Strict Side Grasp Candidate Planner

Date: 2026-07-06

Goal:
Make the side grasp planner choose a realistic gripper pose for demonstration.

Main result:
The planner now uses strict side grasp only.
The gripper roll is fixed to 0 degrees so that the gripper plates stay parallel to the object sides.
Top down and angled grasp candidates are temporarily disabled for this demo stage.

Planner behavior:
- Input is object grasp point from /vlm_robot_tf/grasp_point.
- Only side grasp candidates are generated.
- Side grasp roll is fixed to 0 degrees.
- Pregrasp distance candidates are 0.06 m, 0.08 m, 0.10 m, and 0.12 m.
- The planner evaluates all candidates.
- The selected result is the successful candidate with minimum joint motion cost.
- Joint states are read directly from /joint_states if MoveIt current state timestamp is not usable.
- This prevents planning from a default state and reduces excessive wrist rotation.
- The planner publishes selected grasp pose, selected pregrasp pose, markers, and display trajectory.

Important topics:
- /vlm_robot_tf/grasp_point
- /vlm_moveit/selected_grasp_pose
- /vlm_moveit/selected_pregrasp_pose
- /vlm_moveit/grasp_candidate_markers
- /display_planned_path

Current demo rule:
- strategy must be side.
- roll must be 0.
- gripper plates must be parallel to object sides.
- candidate with minimum joint motion is preferred.
- actual robot execution is not enabled yet.

Current status:
- Strict side grasp marker is working.
- Strict side grasp plan-only trajectory is working.
- Gripper orientation is visually satisfactory in RViz.
- Ready for the next stage: execute safety gate and real robot execution preparation.

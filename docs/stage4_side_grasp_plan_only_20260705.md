# Stage 4 Side Grasp Plan Only Pipeline

Date: 2026-07-05

Goal:
Convert the previous top down pregrasp and grasp logic into a side grasp approach.

Side grasp pose generation:
- grasp pose is generated from the object grasp point.
- pregrasp pose is grasp pose shifted by minus 0.10 m in X.
- approach direction is from pregrasp to grasp along plus X.
- pregrasp and grasp use the same orientation.

Main parameters:
- side_grasp_mode true
- pregrasp_offset_x -0.10
- pregrasp_offset_y 0.0
- pregrasp_offset_z 0.0
- target_offset_x 0.0
- target_offset_y 0.0
- target_offset_z 0.0

Tested target quaternion:
- target_qx 1.000
- target_qy 0.000
- target_qz -0.004
- target_qw -0.002

RViz marker verification:
- red sphere is the grasp target.
- cyan sphere is the pregrasp target.
- blue arrow shows the pregrasp to grasp approach direction.
- TCP X arrow shows the desired gripper tcp direction.

Verified:
- grasp x minus pregrasp x is 0.100 m.
- grasp y equals pregrasp y.
- grasp z equals pregrasp z.
- grasp orientation equals pregrasp orientation.

Added node:
- moveit_plan_side_grasp_cpp_node

Plan only behavior:
- Plan 1 moves from current state to pregrasp using full pose target.
- Plan 2 moves from pregrasp to grasp using Cartesian straight approach.
- The node does not call execute.
- The trajectory is published to display planned path.

Current status:
- Side grasp marker generation is working.
- Side grasp target quaternion application is working.
- Side grasp plan-only trajectory is verified in RViz.
- Real robot execution is not enabled yet.

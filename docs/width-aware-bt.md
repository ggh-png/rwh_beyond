# Experimental width-aware pick BT

`demo_width.xml` is the current default pick tree selected by `bt_demo.yaml`. `demo.xml` retains fixed-width behavior. Place/release remains mode 2.

The installed GraspGenX server returns poses and confidence scores, not jaw widths. This tree estimates aperture from the segmented object point cloud along the selected model pose's local X (finger-closing direction). The source is explicitly `point_cloud_estimate`; this is not a neural width prediction or force control. Partial depth can underestimate hidden geometry; physical success is unverified.

For each candidate the estimator requires at least 100 finite points and uses the 2nd/98th percentile extents, symmetrically about the grasp axis. Degenerate/off-centre estimates are rejected before a pregrasp is selected. Oversized finite estimates are retained in the reported grasp_width but the commanded gap is capped at gripper_open_width (at most the PiPER model limit of 0.10 m). The closing inset is subtracted before capping. The uncapped gap is estimated aperture minus `grasp_width_closing_inset` (default 0.002 m, accepted 0–0.005 m). There is no fixed-width fallback in this tree. Saturation does not establish that an oversized object can actually be grasped. Candidate selection logs `grasp_width_m`, `close_width_m` and `source=point_cloud_estimate`; the action result and BT blackboard expose both widths in metres. The closing target is bound to the selected plan ID and cannot be overridden by a later AttachObject call.

The gripper command uses an explicit closing marker (`path_tolerance` for gripper position -1) so physical timed closing works for widths other than the fixed legacy 0.04 m. Timed completion is not grip detection. Release remains guarded; failed motions retain attachments.

After rebuilding message-dependent packages, restart the affected ROS nodes together. Activating the real BT runner can call its configured exit_teach_mode startup service. No robot motion was run while creating this tree.

Inside the ROS container, with the runner already ready:

```bash
source /root/ros2_ws/install/setup.bash
TREE="$(ros2 pkg prefix rwh_beyond_bt)/share/rwh_beyond_bt/behavior_trees/demo_width.xml"
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  "{mode: 1, behavior_tree: '$TREE', target_text: 'yellow block'}" --feedback
```

The explicit behavior_tree overrides mode selection. To use the original fixed-width tree, explicitly set behavior_tree to the installed `demo.xml` path. Omitting behavior_tree uses `default_demo_bt_xml`, currently `demo_width.xml`.

## Repeated picks

This width tree opts in to `CheckGraspReady release_previous="true"`. At the beginning of the next goal, any previous target is released **at the current pose**: the gripper opens, opening feedback is confirmed, and only then is its planning attachment cleared. This does not move to a placement location. It can drop a held object where the arm currently is. It works after successful picks and after failed transfers. Opening failure retains holding state and aborts the new goal. Unknown attached objects are rejected. The fixed-width `demo.xml` retains its original attachment guard. No physical release was executed while implementing this behavior.

## 닫기 보정값 조정

`bt_demo.yaml`의 `bt_navigator.ros__parameters.bt_motion.grasp_width_closing_inset`을 조절합니다.
단위는 m이며 전체 손가락 간격에서 빼는 양입니다. 한쪽 손가락당 보정량이 아닙니다.
추정 폭이 40mm일 때 0.002는 38mm, 0.004는 36mm의 목표 간격을 만듭니다.
허용 범위는 0~0.005m이며, 범위를 넘으면 후보별 `invalid_observed_width`가 발생합니다.
코드 기본값 0.002m와 YAML에 명시한 실행값은 구분해야 합니다.
현재 YAML은 0.005m이며 허용 상한입니다. [오류 진단](grasp-troubleshooting.md)을 참고하세요.
변경 후 `bt_demo`를 재시작합니다. 이 값은 힘·접촉 제어가 아닙니다.

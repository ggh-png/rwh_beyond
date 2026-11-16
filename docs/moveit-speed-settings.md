# MoveIt speed and planning settings

Edit `grasp_coordinator.ros__parameters` in `rwh_beyond_bringup/config/bt_demo.yaml` (the same file as the default BT paths). Restart the BT coordinator after changing these startup settings. Existing user YAML settings are preserved.

| Parameter | Default | Applies to |
|---|---:|---|
| moveit_velocity_scaling_factor | 0.1 | Home, pregrasp, OMPL place transfer |
| moveit_acceleration_scaling_factor | 0.1 | Same joint-space plans |
| moveit_cartesian_velocity_scaling_factor | 0.1 | Observation, approach, lift, straight place |
| moveit_cartesian_acceleration_scaling_factor | 0.1 | Same Cartesian requests |
| moveit_planning_time | 10.0 s | Home and place OMPL maximum planning time |
| moveit_candidate_planning_time | 2.0 s | Each candidate pregrasp planning attempt |
| moveit_ik_timeout | 0.1 s | Each explicit IK request |

All scales must be finite and in (0,1]. Zero is rejected because MoveIt can interpret it as a default rather than a stop. Planning budgets must be in (0,60] seconds, IK in (0,5]. Client timeouts grow with larger planning budgets. Cartesian resolution, jump checking and collision checking are unchanged.

For example, changing joint-space velocity and acceleration scales to 0.2 permits faster transfers while retaining 0.1 Cartesian approach/lift settings. This is an example, not a verified physical speed profile. No speed increase was applied during implementation. Both velocity and acceleration limits affect timing; doubling a scale does not guarantee a twofold reduction in cycle time. Robot URDF/joint_limits remain the upper limits.

Other cycle-time factors remain separate: driver `piper.speed_percent` in ros_runner/config/runners.yaml, gripper_close_duration (currently 4 s), opening time (3 s), observation_settle_time, number of inference candidates and repeated home/observation moves in the BT. MoveIt scales do not override the hardware driver's own speed setting or shorten gripper timing. Reducing planning time alone can reduce planning success rather than accelerate motion.

## Validation

149 coordinator/geometry/transport regression tests passed. A separate MoveIt instance on ROS domain 182 used the same robot configuration with trajectory execution disabled and no CAN driver/controller. A complete Cartesian path from nominal home to base offsets X=0.10 m, Z=0.10 m had planned duration 2.500540279 s with velocity/acceleration scales 0.1, versus 1.751899923 s at 0.2. Both fractions were 1.0. This verifies request timing propagation, not physical cycle time. The shared YAML retains the previous 0.1 scales.


## 실물 실행 시간 초과 조정 (2026-11-16)

`bt_demo.yaml`의 `move_group.ros__parameters.trajectory_execution`에서 실물 실행 감시 시간을 조절합니다.

```yaml
allowed_execution_duration_scaling: 3.0
allowed_goal_duration_margin: 20.0
```

실행 상한은 계획 궤적 시간 × 3 + 20초입니다. 기존 margin 2초에서 늘렸습니다. 8.275초 상한이었던 같은 계획은 약 26.275초까지 완료를 기다립니다. 피드백 전송부는 시작값 보정 이동(최대 10도, 0.1rad/s)과 목표 오차 0.02rad 이내 0.5초 유지를 요구하므로 짧은 계획에도 추가 시간이 필요할 수 있습니다. 이 변경은 실행 감시·목표 오차 검사를 해제하지 않습니다. 실물 전송부 자체의 계획 시간 + 30초 제한도 유지합니다.

실물 드라이버 제한은 별도 소유자인 `ros_runner/config/runners.yaml`의 `piper.speed_percent`를 10에서 30으로 조정했습니다. MoveIt 배율 0.5와 같은 의미의 수치는 아니며, 실제 추종 성공·작업 시간은 실물 확인이 필요합니다. 기존 MoveIt 배율과 관절 도달 오차는 변경하지 않았습니다.

적용하려면 진행 중인 작업을 마친 후 `bt_demo`와 `piper_moveit`을 중지하고 `piper_moveit`, `bt_demo` 순으로 다시 활성화합니다. BT만 재시작하면 드라이버와 MoveIt 실행 감시 설정은 바뀌지 않습니다. 설정 검증 과정에서는 CAN 드라이버·로봇 이동을 실행하지 않습니다.

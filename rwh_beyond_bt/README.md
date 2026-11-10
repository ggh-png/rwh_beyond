# Behavior trees

- `rwh_beyond_bt`: BT action/condition plugins, execution XML and Groot2 node models.
- `rwh_beyond_navigator`: Nav2 navigator exposing `/run_demo` (`rwh_beyond_msgs/action/RunDemo`).
- `rwh_beyond_bringup`: `bt_demo.launch.py`, loading the navigator and lifecycle manager.

Package names are robot-neutral. The current demo still targets the PiPER model:
its `arm` group, joint1–joint6 zero home, `base_link`, `tcp_link`, and gripper.

Start the simulation stack, then start the navigator in a separate terminal
after MuJoCo and MoveIt are ready:

```bash
ros2 launch moma_bringup simulation.launch.py gui:=false moveit:=true rviz:=false
ros2 launch rwh_beyond_grasp_bridge grasp_pipeline.launch.py
ros2 launch rwh_beyond_bringup bt_demo.launch.py
```

With `ros_runner`, activate only `bt_demo`: it requires `grasp_bridge`, which
requires `piper_sim`. Stopping the simulator also stops its dependents.
Launching does not send a demo goal.

## Plugin layout

`src/action/` contains one source per action. Shared action bases are in
`include/rwh_beyond_bt/action/`. `src/condition/` is reserved for conditions;
the current demo has none. `src/register_nodes.cpp` exports the BT factory entry
point for `rwh_beyond_bt_nodes`.

## Groot2 node model XML

After building and sourcing the workspace, regenerate the palette from the actual
plugin registrations (no ROS nodes are constructed and no motion is requested):

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/behavior_trees/rwh_beyond_tree_nodes.xml
```

Compare and commit the resulting XML when node IDs or ports change. The XML is
installed into `share/rwh_beyond_bt`; import it into Groot2 before opening
`rwh_beyond_bt/behavior_trees/demo.xml`.

The model XML describes node IDs, types and ports. The execution XML defines the
sequence. The shared library supplies executable implementations. The navigator's
`navigator_plugins.xml` separately registers a Nav2 plugin via pluginlib.

## Parameterized action nodes

### SetGripper

```xml
<SetGripper opening="50"/>
<SetGripper opening="60" unit="mm" duration="2.5"/>
<SetGripper opening="{gripper_opening}"/>
```

`opening` is a required **integer total opening width in millimetres**, from 0
to 100. `unit` defaults to `mm`: `opening="50"` requests a total aperture of 50 mm
(0.05 m), `0` closes the fingers and `100` opens to this model's maximum of 100 mm.
`unit="percent"` remains an explicit optional mode for 0..100% opening. These
commands specify width, not force or finger joint angles. The two unit scales
happen to coincide for this 100 mm model.
`duration` defaults to 3 seconds and must be finite in (0, 3600].
The node calls `gripper_controller/follow_joint_trajectory` using joint `gripper`.
Changing this model's gripper geometry requires updating the aperture limit.

### TcpPose: one object per target

`rwh_beyond_msgs/msg/TcpPose` groups position, quaternion and target semantics:

```text
string frame_id "base_link"
geometry_msgs/Pose pose
bool relative false
bool keep_current_orientation false
```

Position is in metres; orientation is a quaternion in x/y/z/w order in
`frame_id`. `relative` affects only position. `keep_current_orientation=true`
ignores the stored orientation and uses the measured TCP orientation instead.
The ROS Quaternion default is identity `(0, 0, 0, 1)`. A directly constructed
message therefore requests identity orientation unless `keep_current_orientation`
is set to true. `MakeTcpPose` chooses retained orientation when quaternion inputs
are omitted.

### MakeTcpPose → MoveTcp

Create a target once and share one typed blackboard entry:

```xml
<Sequence>
  <MakeTcpPose x="0.25" y="0" z="0.35"
               qx="0" qy="0" qz="0" qw="1" pose="{target_pose}"/>
  <MoveTcp pose="{target_pose}"/>
</Sequence>
```

`MakeTcpPose` is a synchronous, local builder: it creates a message and never
sends a motion goal. It requires `x`, `y`, `z` and an output `pose` mapping.
`frame_id` defaults to `base_link`; `relative` defaults to `false`. Supply all
four quaternion inputs to set orientation, or omit all four to set
`keep_current_orientation=true`. Partial/invalid quaternions fail. The builder
writes the output only after validation succeeds.

For a relative lift preserving current orientation:

```xml
<Sequence>
  <MakeTcpPose x="0" y="0" z="0.30" relative="true" pose="{lift_pose}"/>
  <MoveTcp pose="{lift_pose}"/>
</Sequence>
```

An upstream C++ BT node can also directly supply the ROS message:

```cpp
rwh_beyond_msgs::msg::TcpPose target;
target.pose.position.x = 0.25;
target.pose.position.z = 0.35;
target.pose.orientation.w = 1.0;
blackboard->set("target_pose", target);
```

`MoveTcp` now accepts **only** `pose` and optional `tcp_link` (default
`tcp_link`), plus the inherited Nav2 `server_name` and `server_timeout` ports.
The former scalar position/quaternion/frame/relative ports are removed; migrate
old XML by creating a `TcpPose` with `MakeTcpPose` or another typed producer.
A string is not a `TcpPose` object. No custom literal parsing or ROS topic
subscription is implied by the message definition. The `RunDemo` action fields
are unchanged; remote producers would need an explicit bridge into the blackboard.

Positions must be finite and `frame_id`/`tcp_link` nonempty. Explicit quaternion
components must be finite with norm at least 1e-12; normalization works on a
copy and never mutates the blackboard object. TF is required for relative
position or retained orientation; an absolute full pose needs no current-TCP
TF lookup. MoveIt still validates the target frame and pose.

The pose goal uses `arm`, OMPL/RRTConnect, 10 seconds of planning, 0.1 speed and
acceleration scaling, 5 mm position tolerance and 0.01 rad orientation tolerances.
It does not guarantee a straight-line Cartesian path. Invalid runtime inputs,
unavailable required TF, controller/MoveIt application errors and cancellations
return FAILURE. There is no fallback to arbitrary joint targets.

After rebuilding, restart the navigator to load the updated library and
re-import `rwh_beyond_tree_nodes.xml` in Groot2. The default `demo.xml` is now the red-box pick tree; the original fixed
sequence is preserved in `legacy_demo.xml`.

## Verification

`colcon test --packages-select rwh_beyond_bt --event-handlers console_direct+`
uses mock action servers in ROS domain 87, without moving a robot. It checks goal
conversion, typed message construction/reuse/SubTree remapping, absolute/relative
frames, explicit/retained orientation, quaternion normalization without source
mutation, invalid inputs, missing TF and application failure results.

`rwh_beyond_bt/behavior_trees/parameterized_demo.xml` demonstrates the original
home, +0.30 m along base_link Z, close, home, open sequence using the two new
parameterized nodes. It is an optional tree; the navigator defaults to the red-box pick tree.

## 오류 전달

MoveIt/controller 결과의 signed 오류 코드·이름·원문 메시지·발생 컴포넌트와
BT 단계/노드를 root blackboard의 `demo_feedback` 객체로 navigator에 전달합니다.
MoveIt의 action feedback `state`도 `/run_demo` feedback에 포함됩니다.
`RunDemo` 인터페이스가 변경됐으므로 적용 후 navigator와 client를 재시작해야 합니다.
자세한 출력 예제는 [BT 실패 진단](../docs/bt-operation.md#bt-error-reporting)을 참고하세요.

## Octomap 초기화 노드

`<ClearOctomap once="true" timeout="30.0" fresh_clouds="2"/>`를 트리 맨 앞에 배치합니다.
clear 후 새 점군과 Octomap을 확인할 때까지 RUNNING을 반환합니다.
첫 성공 이후 같은 노드 인스턴스에서는 건너뛰며, `once=false`이면 새 실행마다 초기화합니다.
Launch에서는 지도를 초기화하지 않습니다.

## Simulation grasp stages

`MoveObservation` plans to the calibrated MuJoCo observation joint pose.
`GraspStep` calls the `RunGraspStep` Action with `check_ready`, `segment_target`, `build_object_cloud`, `prepare`,
`move_pregrasp`, `approach`, `attach`, or `lift`. Repeated GraspStep instances
execute different stages, not repeated inference; SAM segmentation runs in segment_target; local ROS CPU back-projects Depth and publishes
PointCloud2 in build_object_cloud; GraspGenX inference happens only in prepare.
The default tree enables observation and returns home after a verified lift.
The original MoveHome/OpenGripper/CloseGripper/ClearOctomap nodes remain in the tree.

CloseGripper disables only its own gripper path-position/velocity tracking check
for the expected blocked-close case. Arm tracking and collision checks remain;
the attach stage requires independent physical object contact on both fingers.

See [red-box grasp operation](../docs/red-box-grasp.md). Hardware is unsupported.

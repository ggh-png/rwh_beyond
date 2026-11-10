# Behavior trees

- `rwh_beyond_bt`: BT action/condition plugins, execution XML and Groot2 node models.
- `rwh_beyond_navigator`: Nav2 navigator exposing `/run_demo` (`rwh_beyond_msgs/action/RunDemo`).
- `rwh_beyond_bringup`: `bt_demo.launch.py`, loading MuJoCo, MoveIt and the navigator.

Package names are robot-neutral. The current demo still targets the PiPER model:
its `arm` group, joint1–joint6 zero home, `base_link`, `tcp_link`, and gripper.

Run the simulation stack:

```bash
ros2 launch rwh_beyond_bringup bt_demo.launch.py gui:=false rviz:=false
```

The runner name is `bt_demo`. Launching does not send a demo goal.

## Plugin layout

`src/action/` contains one source per action. Shared action bases are in
`include/rwh_beyond_bt/action/`. `src/condition/` is reserved for conditions;
the current demo has none. `src/register_nodes.cpp` exports the BT factory entry
point for `rwh_beyond_bt_nodes`.

## Groot2 node model XML

After building and sourcing the workspace, regenerate the palette from the actual
plugin registrations (no ROS nodes are constructed and no motion is requested):

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/rwh_beyond_tree_nodes.xml
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

### MoveTcp

```xml
<!-- Absolute position in base_link, metres; retain current TCP orientation. -->
<MoveTcp x="0.25" y="0.0" z="0.35"/>
<!-- Absolute full pose: quaternion orientation in base_link. -->
<MoveTcp x="0.25" y="0.0" z="0.35" qx="0" qy="0" qz="0" qw="1"/>
<!-- Pose values may come from the blackboard. -->
<MoveTcp x="{target_x}" y="{target_y}" z="{target_z}"
         qx="{target_qx}" qy="{target_qy}" qz="{target_qz}" qw="{target_qw}"/>
<!-- Relative offset along base_link axes, not rotated TCP axes. -->
<MoveTcp x="0" y="0" z="0.30" relative="true"/>
<!-- Blackboard values are supported on every input port. -->
<MoveTcp x="{target_x}" y="{target_y}" z="{target_z}"/>
```

`x`, `y`, `z` are required finite metre values. `frame_id` defaults to `base_link`,
`tcp_link` defaults to `tcp_link`, and `relative` defaults to `false`.

`qx`, `qy`, `qz`, `qw` optionally specify the target orientation as a quaternion
in `frame_id` (ROS component order x, y, z, w). Supply all four together or omit
all four to preserve the current TCP orientation. Values must be finite and the
quaternion norm must be at least 1e-12; accepted quaternions are normalized before
being sent to MoveIt. Partial, zero-length or invalid quaternions fail without
sending a goal. Blackboard references that cannot be resolved also fail.

`relative="true"` affects **only position**: the supplied orientation remains an
absolute orientation in `frame_id`, not a rotational increment. TF is required
for relative position or omitted orientation; an absolute full pose does not
need a current-TCP TF lookup. MoveIt still validates the target frame and pose.
This is a MoveIt pose goal, not a guaranteed straight-line Cartesian path. It uses
`arm`, OMPL/RRTConnect, 10 seconds of planning, 0.1 velocity/acceleration scaling,
5 mm position tolerance and 0.01 rad orientation tolerances.

Both nodes also accept Nav2's `server_name` and `server_timeout` ports. Inputs are
read at the start of each invocation. Malformed literal values reject the XML at load time. Invalid runtime inputs
and unavailable transforms return BT FAILURE without sending a goal. MoveIt/controller application errors
and cancellations return FAILURE; unreachable or colliding targets do not fall
back to arbitrary joint targets. Existing fixed-sequence nodes and `demo.xml`
are unchanged. Example coordinates must be checked for the current scene before
execution.

After rebuilding, restart the navigator to load the updated shared library and
re-import `rwh_beyond_tree_nodes.xml` in Groot2. The model XML is editor metadata;
it does not install executable node implementations into the editor.

## Verification

`colcon test --packages-select rwh_beyond_bt --event-handlers console_direct+`
uses mock action servers in ROS domain 87, without moving a robot. It checks goal
conversion, absolute/relative frames, explicit/retained orientation, quaternion
normalization, blackboard inputs, invalid inputs, missing TF and application
failure results.

`rwh_beyond_bt/behavior_trees/parameterized_demo.xml` demonstrates the original
home, +0.30 m along base_link Z, close, home, open sequence using the two new
parameterized nodes. It is an optional tree; the navigator default is unchanged.

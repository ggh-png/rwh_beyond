# Final grasp TCP height filter

Configure `grasp_coordinator.ros__parameters` in `rwh_beyond_bringup/config/bt_demo.yaml`:

```yaml
grasp_height_filter_enabled: true
grasp_height_above_object_min_m: 0.05
grasp_height_above_object_max_m: 0.10
```

The actual grasp TCP after model-to-TCP conversion must be 5–10 cm above the observed object top in base-frame Z. The top is the 99th percentile of the segmented target cloud transformed into the base frame, without the collision box padding. Bounds are inclusive. Candidate poses are filtered, not translated. This affects the pose reached by ApproachGrasp before gripper closing, not the pregrasp starting pose. TCP height is not fingertip clearance: the gripper geometry and configured TCP offset still determine physical contact.

When enabled, the explicit TCP height band replaces the older vertical contact-in-box heuristic, which would otherwise reject deliberately above-object poses. The horizontal target margin, top-down angle, table clearance, IK and collision checks remain active. If no candidate meets all constraints, selection fails before pregrasp motion and reports `grasp_height_above_object` rejection counts. It does not silently widen the height range. Set `grasp_height_filter_enabled: false` to restore selection without this height restriction. Restart bt_demo after editing these startup parameters. No arm or gripper command was sent during implementation.

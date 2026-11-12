# Place transfer recovery after v1.0.0

## Observed failure

The physical place request used base-frame offsets X=0.20 m, Z=0.15 m relative to the configured home TCP. Cartesian planning returned success code 1 but fraction 0.12698412698412698. The incomplete path was rejected and the payload retained. A subsequent pick correctly refused the retained attachment.

The nominal home TCP is approximately (0.198093, 0, 0.225598) m. Its target is (0.398093, 0, 0.375598) m with unchanged orientation.
An execution-disabled MoveIt instance on an isolated ROS domain produced a full Cartesian path and valid collision-checked IK for this target with no payload or environment obstacles. The original failure scene was not saved, so its exact collision/IK cause remains unconfirmed.

## Change

Only the place transfer can fall back from an incomplete Cartesian path to collision-checked IK followed by an OMPL RRTConnect joint-space plan. The endpoint pose and payload attachment stay unchanged. Speed and acceleration scaling remain 0.1. Invalid planning responses, missing IK, failed or zero-duration plans stop without execution. Execution failures never trigger a retry. Release readiness is reset at the start of every place attempt and set only after completed execution.

Observation, approach and lift retain their strict Cartesian behavior. No collision checks are disabled, and no attachment is silently removed.

## Validation on 2026-11-12

A separate MoveIt instance used ROS domain 178, trajectory execution disabled, no CAN driver, and no trajectory controller. A 0.035 m cube at base-frame (0.2981, 0, 0.3006) blocked the straight transfer. The start state was valid; Cartesian fraction was 0.3968253968253968. Calling the actual execute_place implementation then produced a complete 34-point OMPL trajectory; its execution callback only recorded the trajectory and sent no action.

Unit coverage includes complete Cartesian transfers, partial-path fallback, invalid responses, failed IK/plans, zero-duration plans, no retry after execution failure, and release-readiness reset. Actual physical placement with the retained payload remains to be verified. Restart the BT process to load the updated Python implementation; BT activation can invoke the configured exit_teach_mode startup service.

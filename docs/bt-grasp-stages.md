# BT 단계·포트·Action

기본 트리는 `rwh_beyond_bt/behavior_trees/demo.xml`입니다.

| BT 노드 | endpoint | 입력 → 출력 |
|---|---|---|
| `CheckGraspReady` | `/grasp/check_ready` | 모델·backend·기존 attachment 준비 확인 |
| `GraspStep` | `/grasp_step` | `move_home`, `open_gripper`, `move_observation` |
| `ClearOctomap` | MoveIt clear/scene | Octomap 초기화와 새 filtered cloud 확인 |
| `SegmentObject` | `/grasp/segment_object` | 대상 텍스트 → observation_id, mask |
| `BuildObjectCloud` | `/grasp/build_object_cloud` | observation_id, mask → cloud |
| `InferGrasps` | `/grasp/infer_grasps` | observation_id, cloud → candidates |
| `SelectFeasibleGrasp` | `/grasp/select_feasible_grasp` | observation_id, cloud, candidates → plan_id, pregrasp |
| `MoveToPregrasp` | `/grasp/move_pregrasp` | plan_id → 저장된 사전 접근 경로 실행 |
| `ApproachGrasp` | `/grasp/approach` | plan_id → Cartesian 접근 |
| `AttachObject` | `/grasp/attach` | plan_id → 그리퍼 닫기·계획용 부착 |
| `LiftObject` | `/grasp/lift` | plan_id → TCP 리프트 |

`/grasp/*` 단계는 `RunGraspStage`, `/grasp_step`은 `RunGraspStep` 타입입니다.
`SegmentObject`는 선택적 `target_text` 입력 포트도 지원하며 `/run_demo`의 요청별 텍스트가 우선합니다.

```xml
<SegmentObject observation_id="{observation}" mask="{object_mask}"/>
<BuildObjectCloud observation_id="{observation}" mask="{object_mask}" cloud="{object_cloud}"/>
<InferGrasps observation_id="{observation}" cloud="{object_cloud}" candidates="{grasps}"/>
<SelectFeasibleGrasp observation_id="{observation}" cloud="{object_cloud}"
                     candidates="{grasps}" plan_id="{grasp_plan}" pregrasp="{pregrasp}"/>
<MoveToPregrasp plan_id="{grasp_plan}"/>
<ApproachGrasp plan_id="{grasp_plan}"/>
<AttachObject plan_id="{grasp_plan}"/>
<LiftObject plan_id="{grasp_plan}"/>
```

`observation_id`는 한 번의 촬영, `plan_id`는 coordinator에 보관한 실행 경로를 참조합니다.
마스크·점군·후보를 임의로 교체하는 무상태 API가 아닙니다.
촬영 후 30초가 지났거나 관절이 0.02 rad 넘게 바뀌면 재관측을 요구합니다.

`server_timeout="1000"`은 Nav2 Action 요청 응답 대기 시간(ms)이며 전체 모델 추론 제한 시간이 아닙니다.
별도 `MoveTcp`, `MakeTcpPose`, `SetGripper` 등 범용 플러그인의 포트는 노드 모델 XML에서 확인합니다.
기본 시나리오의 관측 이동은 범용 `MoveObservation`이 아닌 `GraspStep command="move_observation"`입니다.

[코드 안내](project-code-map.md) · [RunDemo](bt-operation.md)

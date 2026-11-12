# BT 단계·포트·Action

기본 트리는 `rwh_beyond_bt/behavior_trees/demo.xml`입니다.

| BT 노드 | endpoint | 입력 → 출력 |
|---|---|---|
| `CheckGraspReady` | `/grasp/check_ready` | 모델·backend·기존 attachment 준비 확인 |
| `MoveHome` | `/grasp/move_home` | 공통 home 이동 |
| `OpenGripper` | `/grasp/open_gripper` | YAML에 지정한 폭·시간으로 열기 |
| `MoveObservation` | `/grasp/move_observation` | 선택적 x·y·z·tilt_deg → 관측 이동 |
| `ClearOctomap` | MoveIt clear/scene | Octomap 초기화와 새 filtered cloud 확인 |
| `SegmentObject` | `/grasp/segment_object` | 대상 텍스트 → observation_id, mask |
| `BuildObjectCloud` | `/grasp/build_object_cloud` | observation_id, mask → cloud |
| `InferGrasps` | `/grasp/infer_grasps` | observation_id, cloud → candidates |
| `SelectFeasibleGrasp` | `/grasp/select_feasible_grasp` | observation_id, cloud, candidates → plan_id, pregrasp |
| `MoveToPregrasp` | `/grasp/move_pregrasp` | plan_id → 저장된 사전 접근 경로 실행 |
| `ApproachGrasp` | `/grasp/approach` | plan_id → Cartesian 접근 |
| `AttachObject` | `/grasp/attach` | plan_id → 그리퍼 닫기·계획용 부착 |
| `LiftObject` | `/grasp/lift` | plan_id → TCP 리프트 |

`MoveHome`, `OpenGripper`, `MoveObservation`의 Action 타입은 `RunGraspStep`,
나머지 `/grasp/*` 단계는 `RunGraspStage`입니다. 시나리오 endpoint는 수행할 동작이 고정되어 있으며
요청의 `command` 값으로 다른 동작을 선택하지 않습니다. 이전 `GraspStep` 플러그인과 `/grasp_step` endpoint는 제거했습니다.
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
`MoveObservation`은 coordinator를 호출하며 고정된 심 전용 관절 자세를 사용하지 않습니다.
`x`, `y`, `z`, `tilt_deg`는 각각 독립적으로 지정할 수 있습니다.
좌표는 절대 위치가 아니라 home TCP 기준 base 축 방향 이동량이며, Z=0이나 음수도 지원합니다.
같은 값을 다시 호출하면 현재 위치에 누적하지 않고 동일한 home 기준 목표로 이동합니다.

## XML에서 동작 조정

단계 순서·반복·조건 분기는 XML에서 조합합니다. 각 Action 노드의 입력은 리터럴 또는
`{blackboard_key}`로 전달할 수 있습니다. `server_name`으로 endpoint를 교체할 수도 있습니다.
기본 트리는 심·실물이 공유하며, 실제 처리와 충돌 검사는 coordinator에서 수행합니다.

| 노드 | 선택 입력 | 단위·범위 | 생략 시 |
|---|---|---|---|
| `MoveHome` | `home_joint_positions` | joint1~6 순서, 유한한 6개 rad | YAML home |
| `MoveObservation` | `home_joint_positions` | 관측 자세의 home 기준, 유한한 6개 rad | YAML home |
| `MoveObservation` | `x`, `y`, `z` | home TCP 기준 base 축 방향 이동량(m), 부호 허용 | X·Y=0, Z=YAML 관측값 |
| `MoveObservation` | `tilt_deg` | home 로컬 Y 회전(°) | YAML Y 기울임 |
| `MoveObservation` | `settle_time` | 0~30초 | YAML 안정화 시간 |
| `OpenGripper` | `opening` | 0~0.10m | YAML 열림 폭 |
| `OpenGripper` | `duration` | 0 초과~30초 | 기본 3초 |
| 세 노드 공통 | `server_name`, `server_timeout` | endpoint, 요청 응답 대기(ms) | 등록 endpoint, Nav2 기본값 |

```xml
<MoveHome home_joint_positions="0;0;0;0;0;0"/>
<OpenGripper opening="0.10" duration="3.0"/>
<MoveObservation home_joint_positions="0;0;0;0;0;0"
                 x="0.0" y="0.0" z="0.10" tilt_deg="40.0" settle_time="0.5"/>
```

관절 배열의 구분자는 세미콜론입니다. home을 바꿀 때는 `MoveHome`과 `MoveObservation`의
home 기준을 일치시킵니다. XML 값은 해당 실행에만 적용되며 YAML이나 다음 요청의 기본값을 바꾸지 않습니다.
현재 `OpenGripper`는 부착된 물체를 자동으로 놓는 용도가 아니며, 부착 상태에서는 요청을 거부합니다.
XML에서 Nav2의 `SubTree`, `Fallback`, 반복·조건 노드를 조합할 수 있지만, 이미 실행한 파지 단계를
무조건 재시도하면 coordinator의 순서·부착 상태 검사에서 거부될 수 있습니다.

`RunGraspStep` Goal 형식이 확장되었으므로 호출 측과 ROS 노드는 같은 버전의 `rwh_beyond_msgs`로
빌드한 뒤 재시작해야 합니다. 외부 `/run_demo` Goal 형식은 유지됩니다.

[코드 안내](project-code-map.md) · [RunDemo](bt-operation.md)

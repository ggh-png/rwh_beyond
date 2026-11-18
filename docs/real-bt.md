# 설정과 변경

실행 중인 트리는 `/run_demo`의 `behavior_tree` 또는 `rwh_beyond_bringup/config/bt_demo.yaml`의 모드별 기본 경로로 결정됩니다.
**트리 순서는 XML에서**, **공통 기본값은 YAML에서**, **장치별 값은 runner 프로필에서** 바꿉니다.

| 바꾸려는 동작 | 파일과 설정 |
|---|---|
| 집기 트리를 바꾼다 | `bt_demo.yaml`의 `default_demo_bt_xml` 또는 요청의 `behavior_tree`. 기본은 `demo_multiview.xml`, 단일 시점 추정 폭은 `demo_width.xml`, 고정 폭은 `demo.xml`. |
| 이동·촬영·복귀 순서를 바꾼다 | `rwh_beyond_bt/behavior_trees/`의 실행 XML. 노드 입력은 [포트 참조](bt-grasp-stages.md)에서 확인합니다. |
| 기본 관측·배치 자세를 바꾼다 | `bt_motion.joint_poses_file`의 `front`, `left`, `right`, `place`. [관절 자세 저장](width-aware-bt.md#multiview) 참고. |
| 기본 대상 또는 분할 방식을 바꾼다 | `bt_perception.target_text`, `selection_policy`. 한 번만 바꿀 때는 Goal의 `target_text`. |
| 닫기 폭을 바꾼다 | 추정 모드는 `bt_motion.grasp_width_closing_inset`, 고정 모드는 `gripper_close_width`. 공통 열림 상한은 `gripper_open_width`. |
| 배치 시 개방 폭·시간을 바꾼다 | `place.xml`의 `ReleaseObject` 포트 `opening`, `duration`. 현재 0.10m, 3초. |
| 접근 거리와 리프트 높이를 바꾼다 | 우선 실행 XML의 `SelectFeasibleGrasp` 포트 `pregrasp_distance`, `lift_distance`를 확인합니다. 포트를 생략하면 `bt_motion`의 같은 이름 설정을 사용합니다. |
| 관절·직선 경로 속도나 계획기를 바꾼다 | `bt_motion.moveit_*_scaling_factor`, `moveit_*_planner_id`와 `move_group` 설정. [속도 설정](moveit-speed-settings.md) · [후보와 계획](grasp-motion-tuning.md) 참고. |
| 실물 카메라·장착 TF·CAN을 바꾼다 | [runner 장치 프로필](runner-device-settings.md)의 `robot_common.yaml`, `robot_real.yaml`. |
| GPU 추론 서버 주소를 바꾼다 | runner 사용 시 `runners.yaml`의 `inference_connection`. 직접 실행 시 launch의 `segmentation_endpoint`, `endpoint`. |

## 그리퍼 폭

`grasp_width_closing_inset`은 추정된 **전체 손가락 간격**에서 빼는 값입니다. 허용 범위는 **0~0.02m**이며 공통 YAML에는 **0.02m**가 지정돼 있습니다. 예를 들어 추정 폭이 40mm면 최대 열림 폭 제한 적용 전 닫기 목표는 20mm입니다. [폭 계산 및 멀티뷰](width-aware-bt.md)

## 변경 적용

| 수정 대상 | 반영 시점 |
|---|---|
| Navigator가 읽는 XML 또는 저장된 관절 자세 | XML은 다음 요청에서, 자세 파일은 다음 해당 이동에서 다시 읽습니다. 설치본과 편집 파일이 같은지 확인하세요. |
| 공통 YAML, launch 또는 장치 프로필 | 해당 파일을 읽는 BT·카메라·MoveIt 등 관련 runner를 재시작합니다. |
| C++ 또는 ROS 메시지 정의 | 관련 패키지와 호출 측을 다시 빌드한 뒤 재시작합니다. |

[환경과 빌드](project-containers.md) · [실행 순서](grasp-pipeline-guide.md) · [ROS 이름과 타입](ros-interfaces.md)

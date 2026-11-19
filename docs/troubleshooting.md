# 문제 해결

먼저 `/run_demo` Result의 `failed_node`, `error_domain`, `error_code`, `error_source`,
`error_msg`를 함께 확인합니다. 단계별 오류를 숫자 하나만으로 판단하지 않습니다.
[전체 Result 필드](bt-operation.md#feedback-result)

## 증상으로 찾기

| 증상 | 먼저 확인할 것 | 상세 안내 |
|---|---|---|
| `/run_demo`가 보이지 않음 | ROS 환경 source, domain·RMW, BT 활성화·lifecycle 로그 | [빠른 시작](grasp-pipeline-guide.md) |
| SAM·GraspGenX health 실패 | 모델 로그 → 서버 내부 health → SSH 터널 → bridge endpoint | [GPU 서버 운영](inference-server-operation.md) |
| 촬영 대기 또는 timeout | RGB·정렬 Depth·CameraInfo의 timestamp·크기·frame, 최신 관절값 | [데이터 규약](project-data.md) |
| D435 permission·영상 수신 오류 | USB 연결·실제 장치 전달·동일 카메라 중복 실행 | [D435 단독 점검](d435-standalone.md) |
| 물체 점군·TCP 위치가 어긋남 | 장착 TF, 내부 optical TF, 깊이 정렬, 모델→TCP 변환 | [좌표계](project-data.md) |
| `No feasible model grasp` | 후보 탈락 집계와 원본 MoveIt 오류 | [파지 후보 진단](grasp-troubleshooting.md) |
| 접근·리프트의 부분 경로 | 충돌·도달 범위·손목 제약, Cartesian fraction | [BT 실행 흐름](bt-flows.md) |
| 실행 중 timeout | 실제 driver 속도·피드백·MoveIt 실행 감시·전달기 제한 | [속도와 시간](moveit-speed-settings.md) |
| RViz 결과가 없음 | 해당 BT 단계 성공, domain·QoS·Fixed Frame | [RViz](grasp-visualization.md) |
| 새 XML이 로드되지 않음 | 편집 경로와 요청 트리, 저장 여부, XML filter 로그 | [XML 로드 규칙](bt-operation.md#groot-xml-filter) |

## 준비 상태 확인

아래 명령은 ROS 컨테이너에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 action info /run_demo
ros2 service call /sam3/health std_srvs/srv/Trigger '{}'
ros2 service call /graspgenx/health std_srvs/srv/Trigger '{}'
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic echo /feedback/joint_states --once
```

health 성공은 모델 응답 확인이며 카메라·TF·로봇의 준비까지 보장하지 않습니다.
촬영은 같은 timestamp의 RGB·Depth·CameraInfo와 최신 6축 관절값이 필요합니다.

## 집기·배치 실패 후 다음 요청

- 닫기 전 실패: 미부착 대상과 임시 장면 설정을 정리하고 후속 모션을 중단합니다.
- 닫기 시작 이후 또는 부착 후 실패: 자동 개방하지 않습니다. 실제 물체와 Planning Scene을 확인합니다.
- 지정 배치 위치에 놓기: 부착 상태가 정상이라면 `mode: 2`로 배치를 요청합니다.
  저장된 `place` 관절 자세 도착이 확인돼야 `ReleaseObject`가 개방합니다.
- 기본 `demo_multiview.xml`로 다음 pick: `release_previous=true`이므로 남은 대상을
  **현재 위치에서 개방·해제**할 수 있습니다. 이 동작을 원하지 않으면 사용자 트리에서 false로 설정합니다.

닫기는 시작했지만 Planning Scene 부착이 완료되지 않은 상태나 알 수 없는 부착 물체가 있으면
자동 이동을 거부할 수 있습니다. 오류를 없애기 위해 충돌 객체만 임의 삭제하지 말고 물리 상태와
계획 장면을 먼저 일치시킵니다. [상태·취소 규칙](bt-architecture.md)

## 설정을 바꿔도 동작이 같을 때

설정 위치와 재시작 대상을 확인합니다.

| 바꾼 항목 | 반영 방법 |
|---|---|
| 실행 XML·include | 저장 후 다음 요청에서 로드; 실행 중 트리는 유지 |
| `bt_perception`·`bt_motion` | BT 재활성화 |
| CAN·카메라·TCP·장착 TF | 장치 runner와 BT 재활성화 |
| 실물 MoveIt·실행 감시 | `piper_moveit`과 BT 재활성화 |
| 외부 서버 주소·SSH 포트 | 호스트 터널 재시작 후 BT 재활성화 |
| C++·Action·메시지 정의 | 관련 의존·호출 패키지 재빌드 후 ROS 노드 재시작 |

YAML의 관측 기본값보다 XML에 명시한 값이 우선합니다.
[설정 소유권](runner-device-settings.md) · [빌드·링크 문제](project-containers.md#bt-common-link)

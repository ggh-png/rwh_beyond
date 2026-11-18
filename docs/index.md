# BEYOND Grasp

텍스트로 물체를 지정하면 PiPER가 RGB-D 영상에서 대상을 찾고, 집어서 들어 올립니다.
집기와 배치는 별도의 `/run_demo` 요청이며, 시뮬레이션과 실물은 같은 Navigator와 행동 트리(BT)를 사용합니다.

| 찾는 내용 | 읽을 페이지 |
|---|---|
| 실행 환경을 준비하고 첫 집기를 요청하려면 | [빠른 시작](grasp-pipeline-guide.md) |
| 호출이 어디서 시작되고 각 패키지가 무엇을 하는지 보려면 | [코드를 따라 읽기](project-code-map.md) |
| 자세, 그리퍼 폭, 동작 순서, 카메라를 변경하려면 | [설정과 변경](real-bt.md) |
| 토픽·서비스·액션을 연동하거나 확인하려면 | [ROS 인터페이스](ros-interfaces.md) |

## 한 번의 집기 요청

`/run_demo` → **Navigator가 BT 선택** → **RGB-D 촬영** → **SAM 3 분할** →
**GraspGenX 후보 생성** → **BT가 폭·도달 가능성 검사** → **MoveIt 계획·실행** → **그리퍼 닫기·리프트·복귀**

공통 설정의 기본 트리인 `demo_multiview.xml`은 저장된 `front` 자세에서 먼저 시도합니다.
실패하면 `left`·`right` 관측의 점군을 융합해 다시 시도합니다.
배치 요청은 `place.xml`을 실행해 저장된 `place` 자세에서 그리퍼를 엽니다.

## 자주 바꾸는 위치

| 바꾸려는 것 | 먼저 열 파일 |
|---|---|
| 동작 순서와 단계별 입력 | `rwh_beyond_bt/behavior_trees/` 아래 실행 XML |
| 기본 트리, 폭 보정, 모션·인식 설정 | `rwh_beyond_bringup/config/bt_demo.yaml` |
| 후보 판정, 폭 계산, MoveIt 요청 | `rwh_beyond_bt/src/motion_context.cpp` |
| 액션 요청·취소·결과 처리 | `rwh_beyond_navigator/src/demo_navigator.cpp` |
| 실물 카메라 선택·장착·연결 | [runner 장치 프로필](runner-device-settings.md) |

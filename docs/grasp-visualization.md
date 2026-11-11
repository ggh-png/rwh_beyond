# SAM 마스크·객체 점군을 RViz에서 보기

[전체 구조](project-overview.md) · [처음부터 실행](grasp-pipeline-guide.md) · [데이터 형식](project-data.md) · [코드 안내](project-code-map.md)


`SegmentObject`와 `BuildObjectCloud`를 실행하면 실제 결과를 ROS 토픽으로 확인할 수 있다.
아래 결과 토픽은 Reliable / Transient Local / Keep Last 1로 마지막 메시지를 보존한다.
BT가 끝난 뒤 RViz를 켜도 **coordinator가 계속 실행 중이면** 마지막 결과를 받을 수 있다.
노드를 종료하면 캐시는 사라진다. 결과는 매 프레임 추적 영상이 아니라 마지막 관측의 스냅샷이다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/grasp/object_mask` | sensor_msgs/Image | 깊이 경계 침식 후 실제 점군 생성에 사용한 0/1 마스크 |
| `/grasp/object_mask_preview` | sensor_msgs/Image | RViz에서 밝게 보이는 0/255 마스크; 원래 마스크는 수정하지 않음 |
| `/grasp/segmentation_overlay` | sensor_msgs/Image | 동일 촬영 RGB에 선택 영역을 초록색으로 겹친 rgb8 영상 |
| `/grasp/object_cloud` | sensor_msgs/PointCloud2 | 기존 모델 입력과 동일한 카메라 optical frame의 XYZ, m 단위 |
| `/grasp/object_cloud_base` | sensor_msgs/PointCloud2 | 촬영 시각 TF로 base_link에 변환한 표시용 객체 점군 |

`object_cloud_base`는 카메라가 이동한 뒤에도 원래 관측 위치에 표시된다. 오래된 TF를
RViz에서 다시 찾을 필요가 없다. 원본 점군·마스크·시간·GraspGenX 입력은 바꾸지 않는다.
새 분할이 성공하면 이전 base 점군을 비우고, 다음 `BuildObjectCloud` 결과로 갱신한다.
물체를 들어올린 뒤에도 스냅샷은 원래 위치에 남으며 현재 물체 추적 결과를 뜻하지 않는다.

파지 추론·선택 결과도 같은 보존 QoS로 발행한다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/grasp/grasp_candidates` | visualization_msgs/MarkerArray | 모델 후보 방향·상위 점수 |
| `/grasp/selected_grasp` | visualization_msgs/MarkerArray | 선택 TCP 축·점수, pregrasp, 접근 방향 |
| `/grasp/selected_tcp` | geometry_msgs/PoseStamped | 선택한 실제 PiPER TCP 자세 |
| `/grasp/selected_pregrasp` | geometry_msgs/PoseStamped | 선택한 사전 접근 자세 |

파란 화살표는 **검사 전 모델 원점의 +Z 접근 방향**이다. 기본 상위 10개만 그리며 상위
3개 점수를 표시한다. `debug_max_candidates`는 표시 개수만 제한하고 실제 추론·후보
검사 개수는 바꾸지 않는다. 모델 원점은 PiPER TCP와 다르므로 화살표 시작점과 TCP 축은
서로 떨어질 수 있다. 모델→TCP 보정과 flipped 후보 처리는 선택 단계에서 적용한다.

최종 선택 결과는 실제 TCP의 빨강/초록/파랑 XYZ 축, 자홍색 pregrasp 점, 노란색 접근 선과
선택 점수로 표시한다. 접근 선은 pregrasp→TCP 구간이며 OMPL 관절 경로 전체를 나타내지는
않는다. 다음 분할이 성공하면 후보·선택 MarkerArray를 비워 이전 관측과 섞이지 않게 한다.
PoseStamped 두 토픽은 마지막 성공한 선택 값으로 남으므로 기본 화면은 MarkerArray를 사용한다.

## 실행

[최종 실행 가이드](grasp-pipeline-guide.md)에 따라 모델 서버·runner·BT navigator를 준비한다.
빌드 후 기존 coordinator를 재시작해야 새 토픽·QoS가 적용된다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select \
  rwh_beyond_grasp_bridge rwh_beyond_bt moma_bringup
source install/setup.bash
```

### BT launch에서 MoveIt RViz 함께 실행 — 기본

`rwh_beyond_bringup/launch/bt_demo.launch.py`가 navigator·lifecycle manager와 함께
MoveIt/SAM/파지 통합 RViz를 기본 실행한다(`rviz:=true`). RViz 부분만
`rwh_beyond_bringup/moveit_rviz.launch.py`를 include하여 시뮬레이터와 동일한 robot
description·SRDF·kinematics를 사용한다. 로봇 모델과 기본 RViz 설정은 `moma_bringup`의
설정을 참조하며, 시뮬레이터나 move_group을 추가로 시작하지 않는다.

runner의 `piper_moveit_sim`은 `rviz:=false`로 실행하고, **bt_demo_sim이 RViz를 소유**한다.
따라서 새 runner 설정으로 `/ros_runner/activate_bt_demo_sim`을 호출하면 통합 RViz가 한 번만
열리고, bt_demo_sim을 중지하면 그 RViz도 함께 종료된다. 기존 runner는 설정을 다시 읽도록
한 번 재시작한다. 별도 RViz launch 명령은 기본 실행에 필요하지 않다.

수동으로 simulation을 실행한 경우 BT launch를 시작한다. BT launch가 Bridge도 시작하므로
Bridge를 별도로 실행하지 않는다.

```bash
ros2 launch rwh_beyond_bringup bt_demo.launch.py
```

GUI 없이 실행하려면 직접 launch에서는 `rviz:=false`, runner에서는 **시작 전** 다음 값을
지정한다. 과거 `RWH_PIPER_SIM_RVIZ` 대신 BT 화면은 이 변수를 사용한다.

```bash
export RWH_BT_RVIZ=false
```

`RWH_BT_RVIZ`의 기본값은 true다. `piper_moveit_sim` 단독 실행 시에도 RViz는
시작되지 않는다. 사용자 config는 `bt_demo.launch.py rviz_config:=/path/to/config.rviz`로
지정할 수 있다.

화면은 왼쪽 MotionPlanning·Displays, 가운데 로봇/계획 장면, 오른쪽 분할 영상 2개로
배치했다. 목표의 interactive marker가 결과를 가리지 않도록 `MotionPlanning → Planning
Request → Query Goal State`는 기본 해제했다. GUI에서 목표를 드래그할 때 이 항목을 켠다.

RViz를 비활성화해 실행한 기존 그래프에 나중에 창만 추가하려면 아래 보조 launch를 사용할
수 있다. 기본 bt_demo에서 이미 RViz가 열린 경우에는 중복 실행하지 않는다.

```bash
ros2 launch rwh_beyond_bringup moveit_rviz.launch.py
```

### 별도 결과 전용 뷰어 — 선택 사항

같은 ROS_DOMAIN_ID의 ROS 컨테이너 터미널에서 실행한다. 이 launch는 RViz만 열며
시뮬레이터·모델·BT goal을 시작하지 않는다.

```bash
ros2 launch rwh_beyond_grasp_bridge grasp_visualization.launch.py
```

처음에는 결과가 없다. 아래 관측 BT 또는 기본 집기 BT를 실행하면 왼쪽에 마스크·오버레이,
오른쪽에 초록색 객체 점군이 표시된다. Fixed Frame은 `base_link`다.

직접 RViz를 설정할 경우 Image display에 preview/overlay를, PointCloud2 display에
`object_cloud_base`를 연결한다. 세 display 모두 Reliability=Reliable,
Durability=Transient Local, History=Keep Last, Depth=1을 선택한다.
단순 0/1 마스크는 화면에서 거의 검게 보일 수 있으므로 preview 토픽을 사용한다.

### 기존 시뮬레이션 RViz

`moma_bringup/config/depth_planning.rviz`도 갱신했다. 기본 bt_demo가 이 설정의
MoveIt RViz를 실행하므로 마스크·오버레이·점군·파지 결과를 한 화면에서 볼 수 있다. 이미 열린 RViz는 새 config를 다시 불러오거나
재시작한다. 기존 잘못된 `/grasp/red_mask` 설정은 제거했다.

## 집기 없이 관측만 실행

시뮬레이션용 `perception_preview.xml`을 제공한다. **홈 이동·그리퍼 열기·관측 자세 이동은
실행하지만**, 파지 추론·접근·닫기·리프트는 실행하지 않는다. 이미 물체를 들고 있으면
처음 준비 확인에서 거부한다. 두 실제 모델의 health check는 기존과 같이 요구한다.

runner가 활성화되고 navigator lifecycle이 active인 상태에서 다음 goal을 보낸다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  "{behavior_tree: /root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/perception_preview.xml}" \
  --feedback
```

전체 집기는 기존처럼 빈 goal을 보낸다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo "{}" --feedback
```

## 집기 전 파지 후보·선택 결과까지 확인

`grasp_debug.xml`은 관측 자세 이동 후 SAM→점군→GraspGenX→MoveIt IK/충돌·경로 검사와
후보 선택까지만 실행한다. 사전 접근 이동·그리퍼 닫기·리프트는 하지 않는다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  "{behavior_tree: /root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/grasp_debug.xml}" \
  --feedback
```

이 트리가 끝나면 임시 planning scene 상태·미부착 계획은 정리되고 디버그 마커만 보존한다.
전체 집기를 할 때는 기본 빈 goal을 새로 보내 관측·추론·계획을 다시 수행한다.

## 토픽 확인

```bash
ros2 topic info /grasp/object_cloud_base --verbose
ros2 topic echo /grasp/object_cloud_base --once --field width \
  --qos-reliability reliable --qos-durability transient_local
```

토픽이 없으면 coordinator 실행 여부를 확인한다. 토픽은 있지만 메시지가 없으면
`SegmentObject` / `BuildObjectCloud` 실행과 성공 여부를 확인한다. 모델 서버만 켜서는
이 결과 토픽이 발행되지 않는다. RViz와 ROS 노드는 같은 ROS_DOMAIN_ID를 사용해야 한다.

## 2026-11-11 검증

단위 테스트는 RGB/BGR·행 padding 처리, 0/255 표시, 원본 마스크 불변성, 촬영 TF 변환,
늦게 연결한 구독자의 보존 메시지 수신과 새 분할 시 base 점군 초기화를 확인했다.
실제 SAM·MuJoCo 관측 BT 완료 후 구독자를 연결해 다섯 토픽을 수신했으며, 이후 RViz를 열어
마스크·오버레이·점군이 렌더링되고 Global Status가 OK인 것을 확인했다.
이 시험은 집기를 수행하지 않았다. 모델 서버는 유지하고 검증용 ROS/RViz 프로세스는 종료했다.

실제 관측에서 2,779개 점을 확인했다. 표시용 frame은 `base_link`, 원본 frame은
`d405_color_optical_frame`이며, RViz가 세 표시 토픽을 구독하고 QoS가 일치함을 확인했다.
Bridge·perception 관련 Python 테스트는 새 시각화 검사 포함 61개가 통과했다.
[실제 관측 검증 JSON](validation/grasp-rviz-2026-11-11.json)을 참고한다.

### MoveIt 통합 화면 추가 검증

실제 SAM·GraspGenX로 `grasp_debug.xml`을 실행한 뒤 기존 MoveIt RViz만 추가 실행했다.
마스크·오버레이·점군과 함께 모델 후보 10개 및 최종 선택 TCP·pregrasp·점수를 수신·표시했다.
RViz 추가 전후 `move_group` 개수는 1개로 유지되어 중복 시뮬레이션을 시작하지 않음을 확인했다.
배치 조정 후 실제 창에서 MotionPlanning·로봇·두 분할 영상·마커를 함께 확인했으며,
관련 Python 테스트 62개와 문서 strict 빌드가 통과했다. 이 시험은 파지 계획·선택까지만
검증했고 접근·닫기·리프트는 실행하지 않았다.
[통합 검증 JSON](validation/moveit-grasp-debug-2026-11-11.json)을 참고한다.

### BT launch의 RViz 소유·종료 연동

`bt_demo.launch.py`에 기본 활성화되는 RViz include를 추가하고 `piper_sim`의 RViz는 비활성화했다.
launch/runner 단위 검사 5개와 3개 backend subtest가 통과했다. 별도 ROS 도메인에서 실제
`activate_bt_demo` 호출 후 RViz 프로세스 1개와 move_group 1개를 확인했고,
`deactivate_bt_demo` 호출로 RViz가 종료되는 것을 검증했다. 이 생명주기 시험은 이동 goal을
보내지 않았다. 추가 전체 debug BT 시도 중 한 번은 `MoveObservation`의 기존
`START_STATE_INVALID` 판정으로 중단됐으며 충돌·시작 상태 검사를 완화하지 않았다.
[RViz 소유 검증 JSON](validation/bt-rviz-owner-2026-11-11.json)을 참고한다.
재현 스크립트는 `rwh_beyond_bringup/test/bt_rviz_lifecycle.py`이며 비어 있는 별도 ROS 도메인에서 실행한다.

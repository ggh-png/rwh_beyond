# BT 실행과 Groot2

[전체 구조](bt-architecture.md) · [BT 노드 사용법](bt-nodes.md)

## 1. 빌드하고 실행하기

컨테이너 workspace에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --packages-select \
  rwh_beyond_msgs rwh_beyond_bt rwh_beyond_navigator \
  moma_mujoco_ros moma_bringup rwh_beyond_bringup --symlink-install
source install/setup.bash
ros2 launch moma_bringup simulation.launch.py gui:=false moveit:=true rviz:=false
```

MuJoCo·MoveIt이 준비되면 다른 터미널에서 같은 환경을 source하고 navigator를 실행합니다.
GUI를 표시하려면 시뮬레이션 launch에 `gui:=true rviz:=true`를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch rwh_beyond_bringup bt_demo.launch.py
```

`ros_runner`에서 **`bt_demo`만 활성화**하면 `requires: [piper_sim]`에 의해
MuJoCo·MoveIt·RViz가 자동으로 시작됩니다. 이미 실행 중인 `piper_sim`은 재사용합니다.
BT launch 자체는 navigator만 실행하며, runner가 시뮬레이션 의존성을 관리합니다.
**자동으로 demo goal을 보내지는 않습니다.**

```bash
ros2 service call /ros_runner/activate_bt_demo std_srvs/srv/Empty "{}"
# navigator와 사용되지 않는 simulator 의존성 종료
ros2 service call /ros_runner/deactivate_bt_demo std_srvs/srv/Empty "{}"
```

`piper_sim`을 중지하면 이에 의존하는 `bt_demo`도 중지됩니다.
다른 robot backend로 전환할 때도 의존 관계에 따라 navigator가 함께 종료됩니다.
YAML을 갱신한 뒤 기존 ros_runner 프로세스를 재시작해야 새 설정이 적용됩니다.

### 기본 RViz 화면

시뮬레이션은 `scene_source:=depth`와 `moma_bringup/config/depth_planning.rviz`를 사용합니다.

| 표시 | Topic / 데이터 | 기본 상태 |
| --- | --- | --- |
| MotionPlanning | `/monitored_planning_scene`의 로봇·Octomap | 켜짐 |
| Planned Path | `/display_planned_path` | 켜짐 |
| Depth points - robot filtered | `/camera/depth/filtered_points` | 켜짐, cyan |
| Depth points - raw | `/camera/depth/color/points` | 비교용, 꺼짐 |
| RGB camera | `/camera/color/image_raw` | 켜짐 |
| Depth camera | `/camera/depth/image_rect_raw` | 켜짐 |
| Camera pose | `/camera/pose` | 비교용, 꺼짐 |

Octomap은 별도 `/octomap` display가 아니라 MotionPlanning의 Scene Geometry에 표시됩니다.
점군·영상은 센서의 Best Effort QoS에 맞췄습니다. 겹쳐 보이면 Displays에서 점군을 끄고
Octomap만 확인하세요. 최초 계획 전 카메라 영상과 Octomap 수신을 확인하세요.
GUI 옵션은 runner 시작 전 `RWH_PIPER_SIM_GUI`, `RWH_PIPER_SIM_RVIZ` 환경변수로 바꿀 수 있습니다.

| 실행 경로 | 대상 |
| --- | --- |
| `piper_sim` | MuJoCo + MoveIt (GUI, RViz 및 그리퍼 설정 가능) |
| `bt_demo` | `piper_sim` 자동 시작 + BT navigator 및 lifecycle manager |
| `piper_moveit_sim` | MuJoCo + MoveIt |
| `piper_moveit` | 실물 CAN 드라이버 + MoveIt |

## 2. 서버 등록 확인하기

다른 터미널에서 ROS와 workspace 환경을 source한 뒤 확인합니다.

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /run_demo
ros2 param get /bt_navigator plugin_lib_names
```

| 확인 항목 | 기대값 |
| --- | --- |
| Lifecycle 상태 | `active [3]` |
| `/run_demo` 서버 수 | 1개 |
| BT 라이브러리 설정 | `rwh_beyond_bt_nodes` |

시작 시에는 `bt_lifecycle_manager`가 navigator를 configure하고 activate합니다.
XML의 action 노드가 서버에 연결할 수 있어야 활성화가 완료됩니다.

| 시간 설정 | 기본값 | 용도 |
| --- | --- | --- |
| `wait_for_service_timeout` | 30000 ms | MoveIt 등 action server 발견 대기 |
| `bt_loop_duration` | 10 ms | 트리 tick 주기 |
| `default_server_timeout` | 20 ms | goal 응답 대기 |

서버 발견 대기 시간은 로봇 동작 완료 제한 시간이 아닙니다.

## 3. Groot2에서 편집하기

```bash
groot2
```

다음 순서로 파일을 불러옵니다.

1. **노드 모델 가져오기:** `rwh_beyond_bt/behavior_trees/rwh_beyond_tree_nodes.xml`
2. **트리 열기:** `rwh_beyond_bt/behavior_trees/demo.xml` 또는 `parameterized_demo.xml`

위 경로는 `/root/ros2_ws/src/rwh_beyond/` 기준입니다.

### 두 XML은 역할이 다릅니다

| 파일 | 담고 있는 것 | 사용하는 곳 |
| --- | --- | --- |
| `rwh_beyond_tree_nodes.xml` | 노드 ID, 종류, 포트와 기본값 | Groot2 노드 목록 |
| `behavior_trees/*.xml` | 실행 순서와 입력값 | Navigator의 트리 실행 |

```mermaid
flowchart TD
    C["C++ 노드 / providedPorts"] --> L["BT 라이브러리"]
    L --> G["모델 XML 생성기"]
    G --> M["노드 모델 XML"]
    M --> E["Groot2 편집기"]
    E --> T["실행 XML 저장"]
    T --> N["Navigator가 실행"]
    L --> N
```

Groot2는 모델 XML로 포트를 표시합니다. 실행 구현은 navigator가 C++ 라이브러리에서
로드합니다. 현재 launch에는 Groot2 실시간 모니터 연결을 설정하지 않았습니다.

### C++ 노드나 포트를 바꿨을 때

빌드하고 workspace를 source한 뒤 모델 XML을 재생성합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/behavior_trees/rwh_beyond_tree_nodes.xml
```

- **Navigator:** 프로세스를 재시작해 새 라이브러리를 로드합니다.
- **Groot2:** 갱신된 노드 모델 XML을 다시 가져옵니다.

생성기는 factory에 노드를 등록하지만 action 노드를 생성하거나 goal을 보내지는 않습니다.

## 4. 테스트와 검증 범위

```bash
cd /root/ros2_ws
colcon test --packages-select rwh_beyond_bt --return-code-on-test-failure
colcon test-result --test-result-base build/rwh_beyond_bt/test_results
```

| 검증 | 확인한 내용 |
| --- | --- |
| Mock 테스트 13개 | mm 변환, 객체 생성·전달·재사용·SubTree 연결, 위치·자세와 quaternion 정규화 |
| 잘못된 입력 테스트 | quaternion·TF·blackboard 입력 오류, application error 처리 |
| Headless 통합 실행 | lifecycle active, action server 등록, 라이브러리 로딩 |

Mock 테스트는 ROS domain 87에서 실제 로봇 명령 없이 실행합니다.
**MuJoCo 전체 demo 완료와 실물 동작은 아직 검증하지 않았습니다.**

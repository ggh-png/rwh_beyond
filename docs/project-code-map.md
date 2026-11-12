# 코드를 따라 읽기

처음에는 전체 소스를 순서대로 읽기보다 **BT 노드 하나가 어떤 Action을 호출하고,
서버의 어느 함수로 도착하는지** 따라가면 구조를 빨리 파악할 수 있습니다.
경로는 별도 표기가 없으면 ROS 컨테이너 `/root/ros2_ws/src/rwh_beyond` 기준입니다.

## 1. 실행 구성을 먼저 읽기

| 순서 | 파일 | 확인할 내용 |
|---|---|---|
| 1 | `../ros_runner/config/runners.yaml` | `bt_demo_sim` → `piper_sim` 의존성, 환경변수 |
| 2 | `rwh_beyond_bringup/launch/bt_demo.launch.py` | grasp pipeline·RViz include, navigator·lifecycle manager, 공통 `params_file`을 전체 BT 노드에 전달 |
| 3 | `rwh_beyond_bringup/config/bt_demo.yaml` | navigator·bridge·coordinator·실물 관측 이동 공통 설정 |
| 4 | `rwh_beyond_grasp_pipeline/launch/grasp_pipeline.launch.py` | SAM bridge·GraspGenX bridge·coordinator 세 노드 생성 |
| 5 | `rwh_beyond_grasp_pipeline/config/grasp_pipeline.yaml` | 대상, endpoint, 프레임, TCP 보정, 후보 수 |

시뮬레이터·MoveIt 구성은
`../moma-mj-lab/ros2_ws/src/moma_bringup/launch/simulation.launch.py`에서 이어집니다.
현재 BT의 RViz node 생성 코드는 `rwh_beyond_bringup/launch/moveit_rviz.launch.py`이며,
화면 설정은 `../moma-mj-lab/ros2_ws/src/moma_bringup/config/depth_planning.rviz`입니다.

## 2. XML과 C++ 플러그인 읽기

[기본 XML](https://github.com/ggh-png/rwh_beyond/blob/main/rwh_beyond_bt/behavior_trees/demo.xml)을 열고
`SegmentObject`부터 시작하세요.

```text
rwh_beyond_bt/behavior_trees/demo.xml
  <SegmentObject observation_id="{observation}" mask="{object_mask}"/>
     ↓ XML 이름을 BT_REGISTER_NODES로 등록
rwh_beyond_bt/plugins/action/segment_object_action.cpp
     ↓ SegmentObjectAction / providedPorts / on_success
rwh_beyond_bt/include/rwh_beyond_bt/plugins/action/segment_object_action.hpp
     ↓ 공통 GraspActionNode
rwh_beyond_bt/src/bt_grasp_action_node.cpp
     ↓ ROS Action grasp/segment_object
```

`on_success()`가 응답의 `observation_id`와 `object_mask`를 Blackboard 출력 포트에 씁니다.
다음 `BuildObjectCloud`는 같은 Blackboard 값을 입력 포트로 받아 요청에 담습니다.
각 노드의 헤더와 CPP가 분리되어 있고 개별 플러그인 라이브러리로 빌드됩니다.
기존 `rwh_beyond_bt_nodes`는 호환 로더입니다. [C++ 구조 상세](bt-code-style.md)를 참고하세요.

`rwh_beyond_navigator/src/demo_navigator.cpp`는 `/run_demo` goal과 기본 트리·결과 진단을 연결합니다.
`RunDemo.action`은 어떤 트리를 실행할지 지정하고, `RunGraspStage.action`은 각 파지 단계의 데이터를 전달합니다.

## 3. Python에서 단계별 구현 찾기

아래 단계 구현은 `rwh_beyond_grasp_pipeline/rwh_beyond_grasp_pipeline/` 안에 있습니다.
모델 서버 노드는 [모델별 ROS bridge](model-bridges.md)에 별도 위치를 정리했습니다.

| BT 노드 | Action endpoint | 구현의 시작점 |
|---|---|---|
| `CheckGraspReady` | `/grasp/check_ready` | `grasp_coordinator.py::check_ready()` |
| `SegmentObject` | `/grasp/segment_object` | `perception_stages.py::segment_target()` |
| `BuildObjectCloud` | `/grasp/build_object_cloud` | `perception_stages.py::build_object_cloud()` |
| `InferGrasps` | `/grasp/infer_grasps` | `inference_stages.py::infer_grasps()` |
| `SelectFeasibleGrasp` | `/grasp/select_feasible_grasp` | `selection_stages.py::select_feasible_grasp()` |
| `MoveToPregrasp` | `/grasp/move_pregrasp` | `motion_stages.py::execute_trajectory()` |
| `ApproachGrasp` | `/grasp/approach` | `motion_stages.py::execute_cartesian()` |
| `AttachObject` | `/grasp/attach` | `scenario_motion.py::attach()` |
| `LiftObject` | `/grasp/lift` | `scenario_motion.py::lift()` |

공통 진입점은 `grasp_coordinator.py::execute_stage()`입니다. endpoint별 ActionServer를 만들고,
관측/계획 ID·입력 일치·실행 순서 검증 후 해당 모듈 함수를 호출합니다.
`MoveHome`, `MoveObservation`, `OpenGripper`, `CloseGripper`는 위 파지 단계와 별도로
기존 C++ MoveIt·그리퍼 노드를 사용합니다. [전체 노드 목록](bt-nodes.md)을 참고하세요.

## 4. SegmentObject 요청 하나 끝까지 추적하기

```text
segment_object_action.cpp
  → grasp/segment_object (RunGraspStage)
  → grasp_coordinator.py::execute_stage()
  → perception_stages.py::segment_target()
      snapshot() → 새 RGB/Depth/Info
      lookup_transform() → 촬영 시각 TF
      rwh_beyond_sam3_bridge.action_client::segment_via_action()
        /segment_image Action → sam3_bridge.action_server
        segmentation_client.py::segment_image() → zmq_client.py::request()
          → SAM 컨테이너 /opt/segmentation/server.py
          → 공식 Sam3Processor
      관측 context와 observation_id 저장
      visualization.py::publish_segmentation()
  → Action result.object_mask / observation_id
  → C++ on_success() → Blackboard
```

다음 `BuildObjectCloud`는 같은 context에서 `depth_converter.py::depth_xyz()`를 호출합니다.

## 5. InferGrasps의 두 Action을 구분하기

```text
BT InferGrasps
  → /grasp/infer_grasps           # RunGraspStage: 관측 ID와 BT 데이터 연결
  → coordinator / inference_stages.py
  → /infer_grasps                 # InferGrasps: 독립 추론 인터페이스
  → rwh_beyond_graspgenx_bridge/inference_action_server.py
      rwh_beyond_model_utils/cloud_converter.py::cloud_xyz()
      zmq_client.py::request() → GraspGenX 5566
      rwh_beyond_model_utils/pose_converter.py::validate_candidates() / quaternion_xyzw()
  → 후보 배열 반환 → coordinator → BT Blackboard
```

`/grasp/infer_grasps`와 `/infer_grasps`는 이름이 비슷하지만 서로 다른 Action입니다.
기본 BT는 POINT_CLOUD 모드이며, 독립 RGBD/Depth 경로는 GraspGenX bridge에서 분기하며 RGBD는 SAM Action을 호출합니다.

## 6. 선택·실행과 시뮬레이터

- `selection_stages.py`: SAM 관측 충돌 형상, 대상 Octomap 제외 영역, 후보별 거절 사유·IK·경로 계획.
- `observed_geometry.py`: 관측 XYZ와 주변 Depth에서 물체 AABB·수평 지지면 추정.
- `motion_stages.py`: IK·Cartesian 경로와 ExecuteTrajectory 실행.
- `scenario_motion.py`: 심/실물 공통 home·관측·계획용 attachment·리프트.
- `gripper_motion.py`: 공통 그리퍼 Action 요청·완료·취소 처리.
- `hardware_coordinator.py`: 실물 준비 확인·피드백·계획 시작 상태 보정.
- `physical_trajectory.py`: 실물 명령 전달과 4초 닫기 완료 처리.
- `visualization.py`: 마스크·overlay·base 점군·후보·선택 marker 발행. 계획 입력을 바꾸지 않음.
- `grasp_coordinator.py`: 공유 상태·락·취소·정리. 실패/취소 시 임시 제외 영역·충돌 허용 복원.
- `../moma-mj-lab/ros2_ws/src/moma_mujoco_ros/moma_mujoco_ros/bridge.py`: 카메라/clock/관절·trajectory·검증 서비스의 ROS 연결.
- 같은 폴더의 `trajectory.py`, `contacts.py`: 시뮬레이션 궤적·접촉 처리.

현재 모델 자세 보정·하향 선택 가정은 `selection_stages.py`를, 실제 제조사 URDF·MoveIt 설정은
`../agx_arm_ros`를 확인합니다. 모델의 파지 점수만으로 로봇을 직접 움직이지 않습니다.

## 7. 무엇을 고칠 때 어디로 가나

| 작업 | 먼저 수정/확인할 곳 |
|---|---|
| 단계 순서 바꾸기 | `rwh_beyond_bt/behavior_trees/*.xml` |
| SAM 대상 바꾸기 | coordinator 파라미터 `target_text`, `target_box`, `selection_point` |
| SAM 추론 API/선택 규칙 | `deploy/sam3-docker/server.py`, `segmentation_core.py` |
| GraspGenX 시작 옵션 | `deploy/graspgenx-docker/config/server.local.yaml`, `deploy/graspgenx-docker/scripts/run_server.py` |
| 파지 후보 수 | `rwh_beyond_grasp_pipeline/config/grasp_pipeline.yaml`: `num_grasps`, `top_k`, `max_candidates` |
| TCP 변환·접근/리프트 거리 | 같은 YAML의 `model_to_tcp_*`, `pregrasp_distance`, `lift_distance` |
| 파지 선택 조건 | `selection_stages.py` |
| BT 노드 추가 | `plugins/action/*.cpp`, `include/.../plugins/action/*.hpp`, CMake, XML 모델 |
| Action 데이터 추가 | `rwh_beyond_msgs/action/`, 관련 서버·C++ 노드 함께 변경 |
| RViz 표시 추가 | `visualization.py` + `../moma-mj-lab/ros2_ws/src/moma_bringup/config/depth_planning.rviz` |

## 8. 빌드와 확인

ROS 컨테이너에서 실행합니다. 아래는 관련 코드 변경 후의 빌드이며 매 실험마다 필요하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-up-to \
  rwh_beyond_bringup rwh_beyond_grasp_bridge ros_runner
source install/setup.bash
```

Action 타입 변경 후 서버·클라이언트·runner를 재시작합니다. 모델 서버 코드를 바꾸지 않았다면
GPU 모델 서버를 재시작할 필요는 없습니다.

```bash
# 문서 검사: 실제 집기/이동을 시작하지 않음
cd /root/ros2_ws/src/rwh_beyond
python3 -m mkdocs build --strict
```

Python/BT 테스트와 실제 GPU·물리 통합 시험은 범위가 다릅니다.
`rwh_beyond_grasp_bridge/test/test_*.py`는 단위/프로토콜 검사,
`sam_pick_integration.py`, `sam_runner_pick_integration.py`는 모델·시뮬레이터가 필요한 통합 시험입니다.
실제 시험은 별도 ROS domain에서 실행하며 [기존 검증 방법](bt-grasp-stages.md)을 따릅니다.


이전 패키지 안의 Python 파일은 호환 import 래퍼입니다. 실제 구현을 수정할 때는
[소유 패키지와 API 목록](model-bridges.md)에서 위치를 확인하세요.

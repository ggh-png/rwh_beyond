# 코드를 따라 읽기

## 패키지

| 패키지 | 책임 |
|---|---|
| `rwh_beyond_bringup` | launch, 공통 BT YAML, D405, 실물 MoveIt 구성 |
| `rwh_beyond_msgs` | RunDemo·RunGraspStage·RunGraspStep·SegmentImage·InferGrasps |
| `rwh_beyond_navigator` | `/run_demo`, 대상 텍스트 전달, 결과·취소 진단 |
| `rwh_beyond_bt` | BT 플러그인, 공통 `demo.xml` |
| `rwh_beyond_grasp_pipeline` | 관측·점군·추론 요청·후보 선택·경로 실행 |
| `rwh_beyond_sam3_bridge` | `/segment_image`와 SAM 서버 연결 |
| `rwh_beyond_graspgenx_bridge` | `/infer_grasps`와 GraspGenX 서버 연결 |
| `rwh_beyond_model_utils` | RGB·Depth·점군·pose·ZMQ 공통 변환 |

## 주요 실행 파일

1. `rwh_beyond_bringup/launch/bt_demo.launch.py`: mode·시계·공통 YAML·기본 XML 선택.
2. `rwh_beyond_grasp_pipeline/launch/grasp_pipeline.launch.py`: 두 bridge와 mode별 coordinator 생성.
3. `rwh_beyond_navigator/src/demo_navigator.cpp`: Action goal을 BT 실행으로 연결.
4. `rwh_beyond_bt/behavior_trees/demo.xml`: 실제 공통 시나리오.
5. `rwh_beyond_bt/plugins/action/`: 역할별 ROS Action 클라이언트 플러그인.

## Coordinator 모듈

아래 파일은 `rwh_beyond_grasp_pipeline/rwh_beyond_grasp_pipeline/`에 있습니다.

| 파일 | 책임 |
|---|---|
| `grasp_coordinator.py` | ROS 연결·단계 dispatch·관측/계획 수명·취소·정리, 심 backend |
| `hardware_coordinator.py` | 실물 준비·피드백·계획 시작 상태 보정 |
| `perception_stages.py` | 요청별 텍스트·RGB-D·촬영 TF·분할·점군 |
| `inference_stages.py` | `/infer_grasps`에 객체 점군 전달 |
| `selection_stages.py` | 장면·TCP 보정·IK·충돌·경로 검사·후보 선택 |
| `motion_stages.py` | MoveIt IK·Cartesian·ExecuteTrajectory |
| `scenario_motion.py` | 공통 home·관측·계획용 attachment·리프트 |
| `gripper_motion.py` | 공통 그리퍼 trajectory Action·완료·취소 |
| `physical_trajectory.py` | 실물 명령 전달·시작 연결·시간 기반 닫기 |
| `planning_state.py` | URDF 관절 한계·계획용 상태 복사본 보정 |
| `observed_geometry.py` | 관측 점군과 수평 지지면에서 충돌 형상 추정 |
| `target_cloud_filter.py` | 대상 영역을 계획용 점군에서 제외 |
| `visualization.py` | 마스크·점군·후보·선택 TCP 디버그 출력 |

카메라 모델과 MoveIt robot description은 `moma-mj-lab`이 관리합니다.
PiPER driver는 `agx_arm_ros`, runner 설정은 `ros_runner/config/runners.yaml`에 있습니다.

[BT 구조](bt-architecture.md) · [데이터 형식](project-data.md)

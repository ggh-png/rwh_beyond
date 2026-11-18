# ROS 인터페이스

외부 앱은 **`/run_demo`**로 집기·배치를 요청합니다. BT는 모델 브릿지와 MoveIt을 호출하고 센서와 관절 피드백을 읽습니다. 아래 이름은 루트 네임스페이스의 기본 구성 기준이며, 실제 이름은 launch·namespace·remap에 따라 달라집니다.

## 액션

| 이름 | 타입 | 호출 → 제공 · 용도 |
|---|---|---|
| `/run_demo` | `rwh_beyond_msgs/action/RunDemo` | 외부 앱 → Navigator · 집기·배치, 피드백·결과·취소 |
| `/segment_image` | `rwh_beyond_msgs/action/SegmentImage` | BT → SAM 3 브릿지 · RGB와 대상 조건으로 마스크 생성 |
| `/infer_grasps` | `rwh_beyond_msgs/action/InferGrasps` | BT → GraspGenX 브릿지 · 점군으로 파지 후보 생성 |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | BT → MoveIt · 계획한 궤적 실행 |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt → 팔 controller · 관절 궤적 실행 |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | BT → 그리퍼 controller · 전체 손가락 간격 제어 |

집기·배치 요청은 [RunDemo Goal·Feedback·Result](bt-operation.md), 모델 입력 형식과 응답은 [모델 브릿지](model-bridges.md)에 있습니다. 모델 액션을 사용하는 XML 노드에는 `server_name` 포트가 있습니다. 실물의 팔·그리퍼 controller 액션은 `physical_trajectory.py`가 제공합니다.

## 토픽

| 이름 | 타입 | 발행 → 구독 · 용도 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 카메라 → `CaptureRgbd` · RGB |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 카메라 → `CaptureRgbd` · RGB에 정렬한 깊이 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 카메라 → `CaptureRgbd` · 깊이 역투영용 보정값 |
| `/feedback/joint_states` | `sensor_msgs/msg/JointState` | 로봇/시뮬레이터 → BT·실물 전송 · 현재 관절 및 그리퍼 상태 |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | 실물 궤적 전송 → PiPER 제어 · 관절 목표 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 로봇·카메라 TF → BT·MoveIt · 좌표 변환 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이터 → `use_sim_time`을 쓰는 ROS 노드 |
| `/camera/depth/color/points`, `/camera/depth/planning_points` | `sensor_msgs/msg/PointCloud2` | 실물 카메라 점군 → 대상 제외 필터 → MoveIt 장애물 지도 |
| `/camera/depth/filtered_points` | `sensor_msgs/msg/PointCloud2` | BT의 Octomap 갱신 확인 입력. 공통 설정은 `octomap_enabled: false`. |
| `/grasp/object_mask`, `/grasp/object_mask_preview`, `/grasp/segmentation_overlay` | `sensor_msgs/msg/Image` | BT → RViz·관찰 도구 · 분할 결과 |
| `/grasp/object_cloud`, `/grasp/object_cloud_base`, `/grasp/fused_object_cloud` | `sensor_msgs/msg/PointCloud2` | BT → RViz·관찰 도구 · 객체 및 융합 점군 |
| `/grasp/selected_tcp`, `/grasp/selected_pregrasp` | `geometry_msgs/msg/PoseStamped` | BT → RViz·관찰 도구 · 선택 자세 |
| `/grasp/grasp_candidates`, `/grasp/selected_grasp` | `visualization_msgs/msg/MarkerArray` | BT → RViz · 후보·선택 시각화 |

BT의 이미지 입력 설정은 `bt_perception.rgb_topic`, `depth_topic`, `info_topic`입니다. 카메라별 remap과 TF는 [D405](d405-sensors.md)와 [D435](d435-bt-demo.md)를 참고하세요.

## 서비스

| 이름 | 타입 | 호출 → 제공 · 용도 |
|---|---|---|
| `/sam3/health`, `/graspgenx/health` | `std_srvs/srv/Trigger` | BT·운영자 → 모델 브릿지 · 모델 연결 확인 |
| `/compute_ik`, `/compute_fk` | `moveit_msgs/srv/GetPositionIK`, `GetPositionFK` | BT → MoveIt · 역·순기구학 |
| `/plan_kinematic_path`, `/compute_cartesian_path` | `moveit_msgs/srv/GetMotionPlan`, `GetCartesianPath` | BT → MoveIt · 관절 계획과 직선 경로 |
| `/get_planning_scene`, `/apply_planning_scene` | `moveit_msgs/srv/GetPlanningScene`, `ApplyPlanningScene` | BT → MoveIt · 장면 조회·충돌 형상·부착 갱신 |
| `/move_group/get_parameters` | `rcl_interfaces/srv/GetParameters` | BT → MoveIt · 로봇 설명 읽기 |
| `/clear_octomap` | `std_srvs/srv/Empty` | BT → MoveIt · Octomap 사용 시 지도 초기화 |
| `/grasp/set_grasp_region`, `/simulation/set_grasp_region` | `rwh_beyond_msgs/srv/SetGraspRegion` | BT → 실물 점군 필터 또는 시뮬레이터 · 계획 점군에서 대상 영역 제외 |
| `/grasp/hardware_transport_ready` | `std_srvs/srv/Trigger` | BT → 실물 궤적 전송 · 전송 준비 확인 |
| `/control_enable` | `std_srvs/srv/SetBool` | 실물 궤적 전송 → 장치 제어 · 제어 허용 |
| `/ros_runner/activate_bt_demo_sim`, `/ros_runner/activate_bt_demo` | `std_srvs/srv/Empty` | 운영자 → ROS runner · 심·실물 의존 프로세스 실행 |

실행 환경의 이름과 타입은 `ros2 topic list -t`, `ros2 service list -t`, `ros2 action list -t`로 확인할 수 있습니다. 이 프로젝트가 정의하는 타입은 `rwh_beyond_msgs`에 있습니다.

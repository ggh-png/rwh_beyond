# Runner의 PiPER·RealSense 설정

장치 실행 설정은 **`ros_runner/config/robot_common.yaml`**, **`robot_real.yaml`**,
**`robot_sim.yaml`**에서 관리합니다. 각 값은 `robot_settings.ros__parameters` 아래에 있습니다.
`runners.yaml`의 `robot_profiles`에 세 파일의 경로를 지정하며 상대 경로는 `runners.yaml` 기준입니다.
`inference_connection`은 계속 `runners.yaml`에 있습니다.
수동 `export` 없이 각 runner를 활성화할 때 YAML을 읽습니다.

## PiPER

`robot_real.yaml`의 `robot_settings.ros__parameters.piper`:

```yaml
piper:
  can_port: can0
  effector_type: agx_gripper
  auto_enable: false
  speed_percent: 30
  auto_control_gate: true
  fw_version: ""
  execution_transport: feedback
  camera_model: true
  start_sensors: true
  octomap: true
  use_rviz: true
```

| 항목 | 의미 |
|---|---|
| `can_port` | 실제 로봇 CAN 인터페이스. 장치가 CAN이고 UP인지 확인 후 실행 |
| `effector_type` | `agx_gripper` 또는 `none`. 집기 BT는 그리퍼가 필요 |
| `auto_enable` | driver 시작 시 모터 자동 활성화. 현재 false |
| `speed_percent` | driver 속도 설정, 정수 1~100 |
| `fast_mode` | driver fast mode, 기본 false |
| `pub_rate` | driver 피드백 발행 주기(Hz), 기본 200 |
| `enable_timeout` | 모터 활성화 서비스 시간 제한(초), 기본 5 |
| `gripper_default_effort` | 그리퍼 기본 노력 값, 기본 1 |
| `auto_control_gate` | 공식 transport의 제어 gate 관리 |
| `fw_version` | 필요한 경우 firmware profile 지정. 빈 값은 launch 기본 동작 |
| `tcp_offset` | flange → TCP의 XYZ(m)·RPY(rad), 6개 값 |
| `execution_transport` | `feedback` 또는 `official`. 실물 BT는 feedback 필요 |
| `camera_model` | MoveIt의 카메라 장착 모델 포함 여부 |
| `start_sensors` | PiPER MoveIt이 센서를 함께 시작할지 여부 |
| `octomap` | 깊이 점군 기반 Octomap 사용 여부 |
| `use_rviz` | 실물 `bt_demo` 활성화 시 통합 RViz 사용 여부 |

`piper_moveit`과 동일 backend 별칭은 같은 설정을 사용합니다.
`tcp_offset`은 `robot_common.yaml`에 있으며 `piper_sim`·`bt_demo_sim`·`bt_demo`에도 전달해 로봇 모델과 BT 화면의 TCP를 맞춥니다.
모터 자동 활성화와 움직임은 설정값에 영향을 받습니다. 현재 기본값은 자동 활성화 false를 유지합니다.

## RealSense 모델 선택과 D405

`robot_real.yaml`의 `robot_settings.ros__parameters.realsense`:

```yaml
realsense:
  model: d435  # d405로 변경하면 D405 사용
  enabled: true
  serial: ""
  profile: 640x480x15
  publish_mount_tf_standalone: true
```

| 항목 | 의미 |
|---|---|
| `model` | 실물 파지 카메라 선택: `d405` 또는 `d435` |
| `enabled` | 선택한 실물 카메라 driver 사용 여부 |
| `serial` | 사용할 카메라 serial. 빈 문자열은 driver의 장치 선택 사용 |
| `profile` | RGB·Depth 공통 `너비x높이xFPS`. 실제 장치가 지원하는 조합 사용 |
| `publish_mount_tf_standalone` | `d405` 단독 runner에서만 명목 장착 TF 발행 여부 |

숫자 serial도 `serial: "001234567890"`처럼 문자열로 작성합니다.
이 설정은 PiPER에 포함한 D405와 `d405` 단독 runner에서 공유합니다.
PiPER와 함께 실행하면 robot_state_publisher가 mount TF를 담당하므로 단독 TF 설정은 사용하지 않습니다.

정렬 Depth·CameraInfo·공통 토픽·SDK 내부 TF는 파지 pipeline의 입력 규약을 유지합니다.
공통 `rwh_beyond_bringup/config/bt_demo.yaml`의 `d405`와 `/camera/d435` 노드 섹션에서
동기화·깊이 정렬·점군·QoS 등 SDK 파라미터를 설정할 수 있습니다.
시리얼·해상도는 `robot_real.yaml` 및 명시적 launch 인자가 우선합니다.
이 설정을 바꾸면 인식 입력이나 TF가 사라질 수 있으므로 변경 전 장치 지원 여부를 확인합니다.
노출·필터 등 여기에 없는 SDK 옵션은 카메라 SDK가 지원하는 ROS 파라미터로 추가할 수 있습니다.

## 심 실행

`robot_sim.yaml`의 `robot_settings.ros__parameters.simulation`:

```yaml
simulation:
  gui: true
  camera_rate: 5.0
  gripper_contact_force_threshold: 1.0
  bt_rviz: true
```

`gui`는 MuJoCo 창, `camera_rate`는 심 카메라 주기(Hz), `bt_rviz`는 심 BT의 통합 RViz입니다.
`gripper_contact_force_threshold`는 시뮬레이터의 접촉 상태 서비스 설정이며
현재 집기 BT의 물체 잡힘 성공 판정을 활성화하는 옵션은 아닙니다.
실물 RealSense profile은 심 영상 해상도 설정이 아닙니다.

## 변경 반영

1. 해당 공통·실물·시뮬레이션 프로필의 값을 수정합니다.
2. 관련 BT와 장치 runner를 비활성화합니다.
3. 장치와 BT를 재활성화합니다.

장치 runner가 이미 실행 중이면 BT만 재활성화해도 장치가 다시 시작되지는 않습니다.
예를 들어 실물 카메라 profile·TCP·CAN 변경은 `bt_demo`와 `piper_moveit`을 중지한 뒤 다시 활성화합니다.
심 TCP·카메라 주기 변경은 `bt_demo_sim`과 `piper_sim`을 다시 시작합니다.
단독 카메라 설정은 `d405` runner를 다시 시작합니다.
현재 개발 환경의 symlink 설치에서는 YAML 값만 바꾸면 재빌드는 필요 없습니다.
일반 설치에서는 설치된 설정을 수정하거나 변경 후 해당 패키지를 다시 빌드합니다.

설정은 상속된 `RWH_PIPER_*`, `RWH_D405_*` 환경변수보다 우선합니다.
커스텀 `config_yaml`을 쓰면 그 파일의 `robot_profiles`가 가리키는 프로필을 읽습니다.
공통과 모드 프로필에 같은 항목을 중복 지정하면 실행이 거부됩니다.

## 실행 설정과 보정값의 경계

| 설정 | 원본 |
|---|---|
| PiPER·카메라 runtime | `ros_runner/config/robot_real.yaml` |
| 공통 TCP / 심 GUI·주기 및 MoveIt 실행 감시 | `robot_common.yaml` / `robot_sim.yaml` |
| 외부 모델 SSH·포트 | `runners.yaml`의 `inference_connection` |
| 관측 높이·기울임 | 선택한 XML의 두 `MoveObservation`; 기본 `demo_width.xml` |
| 그리퍼 폭·닫기 시간·후보 수 | `rwh_beyond_bringup/config/bt_demo.yaml` |
| 접근·리프트 거리 | XML 명시값 우선, 생략 시 `bt_motion` YAML |
| 실물 카메라 장착 위치·회전 | runner의 `d405.mount` / `d435.mount` |
| 기본 카메라 모델·intrinsics | `moma-mj-lab`의 모델 파일 및 실물 장치 SDK |

실물 runner 장착값은 기본 카메라 모델의 변환을 덮어쓰며 MoveIt·TF·RViz에 함께 전달됩니다.
로봇 기구 모델과 센서 내부 보정은 각 모델·SDK에서 관리합니다.
`tcp_offset`과 파지 모델의 `model_to_tcp_*` 보정은 서로 다른 변환입니다.

[실행 가이드](grasp-pipeline-guide.md) · [D405와 TF](d405-sensors.md) ·
[실물 제어](piper-hardware-moveit.md) · [외부 모델 연결](piper-runtime.md)

## 실물 BT 시작 시 티칭 종료

`runners.yaml`의 다음 설정은 실물 `bt_demo`에만 적용됩니다.

```yaml
bt_demo_startup:
  exit_teach_mode: true
  delay_seconds: 10.0
  service_timeout_seconds: 15.0
```

BT launch 시작 후 10초가 지나면 `/exit_teach_mode`를 한 번 호출합니다.
이 서비스는 팔 home 이동과 모터 활성화를 수행할 수 있으며 `piper.auto_enable`과 별개입니다.
`exit_teach_mode: false`로 자동 호출을 끌 수 있습니다.
대기 중 BT를 중지하면 예약도 취소됩니다. 이미 전송한 Empty 요청은 취소할 수 없습니다.
서비스 대기/응답 시간이 제한을 넘으면 호출 클라이언트를 종료하고 자동 재시도 없이 BT를 유지합니다.
로그와 실제 arm 상태를 확인한 뒤 `/run_demo`를 호출합니다.
설정값은 다음 활성화부터 적용됩니다.

실물 `bt_demo`가 RViz를 함께 시작하고 종료합니다. 의존 runner인 `piper_moveit`은
RViz를 따로 시작하지 않아 창이 중복되지 않습니다. `piper.use_rviz: true`가 기본값입니다.

## D435와 장착 TF

D435의 `serial`, `depth_profile`, `color_profile`, 단독 `use_rviz`는 `d435` 아래에 설정합니다.
현재 설정은 깊이 480x270x6, 컬러 424x240x6입니다. 장치·USB 대역폭에 맞는 프로파일을 선택합니다.
BT 통합 RViz는 `piper.use_rviz`를 사용합니다.

```yaml
d435:
  serial: "143322072540"  # 사용할 실제 장치 번호로 변경
  depth_profile: 480x270x6
  color_profile: 424x240x6
  use_rviz: true
  mount:
    parent_frame: link6
    translation_m: [-0.09, 0.05, 0.075]
    rotation_rpy_deg: [0.0, -90.0, 0.0]

d405:
  mount:
    parent_frame: link6
    translation_m: [-0.06, 0.0, 0.065]
    rotation_rpy_deg: [180.0, -90.0, 0.0]
```

장착 위치는 부모 링크 → SDK 깊이 원점의 XYZ(m), 회전은 roll/pitch/yaw(도)입니다.
현재 D435 값은 사용자 조정값이며 실측 보정 완료를 뜻하지 않습니다.
선택한 카메라의 mount가 MoveIt·robot_state_publisher·BT RViz에 함께 적용됩니다.
`d405.mount`는 D405 단독 정적 TF에도 전달됩니다.

BT에는 `realsense.model`로 선택한 센서를 포함하는 `piper_moveit`을 사용합니다.
단독 `d435` runner를 먼저 켤 필요는 없습니다.
[ D435 BT 연결](d435-bt-demo.md) · [D435 단독 점검](d435-standalone.md) · [D405 mount](d405-runner-mount.md)

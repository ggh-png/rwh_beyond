# Runner의 PiPER·RealSense 설정

장치 실행 설정은 **`ros_runner/config/runners.yaml`**에서 관리합니다.
`inference_connection`은 외부 모델 서버, `piper`·`realsense`·`simulation`은 장치 실행 설정입니다.
수동 `export` 없이 각 runner를 활성화할 때 YAML을 읽습니다.

## PiPER

```yaml
piper:
  can_port: can0
  effector_type: agx_gripper
  auto_enable: false
  speed_percent: 10
  auto_control_gate: true
  fw_version: ""
  tcp_offset: [0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]
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
| `auto_control_gate` | 공식 transport의 제어 gate 관리 |
| `fw_version` | 필요한 경우 firmware profile 지정. 빈 값은 launch 기본 동작 |
| `tcp_offset` | flange → TCP의 XYZ(m)·RPY(rad), 6개 값 |
| `execution_transport` | `feedback` 또는 `official`. 실물 BT는 feedback 필요 |
| `camera_model` | MoveIt의 카메라 장착 모델 포함 여부 |
| `start_sensors` | PiPER MoveIt이 센서를 함께 시작할지 여부 |
| `octomap` | 깊이 점군 기반 Octomap 사용 여부 |
| `use_rviz` | 실물 `bt_demo` 활성화 시 통합 RViz 사용 여부 |

`piper_moveit`과 동일 backend 별칭은 같은 설정을 사용합니다.
`tcp_offset`은 `piper_sim`·`bt_demo_sim`·`bt_demo`에도 전달해 로봇 모델과 BT 화면의 TCP를 맞춥니다.
모터 자동 활성화와 움직임은 설정값에 영향을 받습니다. 현재 기본값은 자동 활성화 false를 유지합니다.

## RealSense D405

```yaml
realsense:
  enabled: true
  serial: ""
  profile: 640x480x15
  publish_mount_tf_standalone: true
```

| 항목 | 의미 |
|---|---|
| `enabled` | 실제 D405 driver 사용 여부 |
| `serial` | 사용할 카메라 serial. 빈 문자열은 driver의 장치 선택 사용 |
| `profile` | RGB·Depth 공통 `너비x높이xFPS`. 실제 장치가 지원하는 조합 사용 |
| `publish_mount_tf_standalone` | `d405` 단독 runner에서만 명목 장착 TF 발행 여부 |

숫자 serial도 `serial: "001234567890"`처럼 문자열로 작성합니다.
이 설정은 PiPER에 포함한 D405와 `d405` 단독 runner에서 공유합니다.
PiPER와 함께 실행하면 robot_state_publisher가 mount TF를 담당하므로 단독 TF 설정은 사용하지 않습니다.

정렬 Depth·CameraInfo·공통 토픽·SDK 내부 TF는 파지 pipeline의 입력 규약을 유지합니다.
노출·필터 등 이 표에 없는 SDK 옵션은 현재 runner 설정 항목으로 추가하지 않았습니다.

## 심 실행

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

1. `runners.yaml`의 해당 값을 수정합니다.
2. 관련 BT와 장치 runner를 비활성화합니다.
3. 장치와 BT를 재활성화합니다.

장치 runner가 이미 실행 중이면 BT만 재활성화해도 장치가 다시 시작되지는 않습니다.
예를 들어 실물 카메라 profile·TCP·CAN 변경은 `bt_demo`와 `piper_moveit`을 중지한 뒤 다시 활성화합니다.
심 TCP·카메라 주기 변경은 `bt_demo_sim`과 `piper_sim`을 다시 시작합니다.
단독 카메라 설정은 `d405` runner를 다시 시작합니다.
YAML 값만 바꾸면 재빌드는 필요 없습니다.
이번 설정 로더를 처음 적용할 때는 빌드 후 runner 프로세스도 한 번 재시작합니다.

설정은 상속된 `RWH_PIPER_*`, `RWH_D405_*` 환경변수보다 우선합니다.
커스텀 `config_yaml`로 runner를 실행하면 장치 설정도 같은 파일에서 읽습니다.

## 실행 설정과 보정값의 경계

| 설정 | 원본 |
|---|---|
| PiPER·카메라 runtime, TCP, 심 GUI/주기 | `ros_runner/config/runners.yaml` |
| 외부 모델 SSH·포트 | 같은 파일의 `inference_connection` |
| 관측 높이·기울임 | `rwh_beyond_bt/behavior_trees/demo.xml` |
| 그리퍼 폭·닫기 시간·후보 수·리프트 | `rwh_beyond_bringup/config/bt_demo.yaml` |
| 카메라 장착 위치·회전·intrinsics 모델 | `moma-mj-lab/config/cameras/d405.yaml` 및 장치 SDK |

카메라 장착 TF와 로봇 기구 모델을 runner에서 중복 정의하지 않습니다.
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
설정값은 다음 활성화부터 적용됩니다. 새 wrapper의 최초 적용에는 runner 재시작이 필요합니다.

실물 `bt_demo`가 RViz를 함께 시작하고 종료합니다. 의존 runner인 `piper_moveit`은
RViz를 따로 시작하지 않아 창이 중복되지 않습니다. `piper.use_rviz: true`가 기본값입니다.

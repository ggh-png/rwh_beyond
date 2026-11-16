# bt_demo에서 D435 사용

현재 `ros_runner/config/runners.yaml`은 다음처럼 설정되어 있습니다.

```yaml
realsense:
  model: d435
  enabled: true

d435:
  serial: "143322072540"
  depth_profile: 480x270x6
  color_profile: 424x240x6
  use_rviz: true
```

`d435.use_rviz`는 센서 단독 runner용이며, BT 통합 RViz는 기존 `piper.use_rviz`를 사용합니다. 현재 USB 2.1 연결에서 실제 수신한 낮은 대역폭 프로파일을 유지했습니다. `realsense.model: d405`로 바꾸면 기존 D405 설정으로 전환합니다. 시뮬레이션 runner는 D405 구성을 유지합니다.

## 실행

runner를 다시 시작한 뒤 `bt_demo`를 활성화합니다. `piper_moveit` 의존성이 D435 드라이버와 D435 손목 모델을 시작하고, BT는 기존 파지 토픽을 구독합니다. `piper_moveit`이 이미 켜져 있으면 해당 runner도 재활성화해야 합니다. 단독 `d435`를 미리 실행할 필요는 없습니다. 실물 backend는 단독 `d435` runner를 먼저 중지해 같은 장치를 이중으로 열지 않습니다.

기존 `bt_demo_startup.exit_teach_mode: true` 설정은 그대로입니다. 사용자가 bt_demo를 활성화하면 기존 시작 절차가 홈 이동·팔 활성화를 요청할 수 있습니다. 이번 검증에서는 bt_demo runner 자체를 활성화하지 않았습니다.

## 입력과 좌표계

D435 전용 launch의 `pipeline_topics:=true`가 다음 토픽으로 연결합니다.

- 컬러: `/camera/color/image_raw`
- 컬러에 정렬한 깊이: `/camera/depth/image_rect_raw`
- 정렬 깊이 CameraInfo: `/camera/depth/camera_info`
- 색상 점군: `/camera/depth/color/points`

원시 깊이는 `/camera/depth/image_raw`로 분리하므로 정렬 깊이와 혼동하지 않습니다. 모델 이름을 D405로 가장하지 않으며 내부 TF는 D435 드라이버가 발행합니다.

`ros_runner/config/runners.yaml`의 `d435.mount`에서 부모 링크와 `d435_link` 위치·회전을 수정합니다. runner 설정이 직접 launch 기본 파일 `rwh_beyond_bringup/config/d435_mount.yaml`보다 우선합니다. 사용자 요청에 따라 기존 D405와 같은 손목 위치를 초기값으로 사용했습니다. `calibrated: false`이며 D435 실측 손눈 보정값은 아닙니다. 이 값은 SDK 깊이 원점인 d435_link 기준이고 장착 나사 원점 기준이 아닙니다. 실제 D435 장착 위치에 맞게 확인해야 합니다.

MoveIt과 BT RViz는 같은 D435 URDF/SRDF를 사용합니다. 카메라 충돌 형상은 D435 크기이며 기존 D405 형상을 제거합니다. 손목의 고정 장착 부품과 카메라 사이만 충돌 예외를 두고, 손가락·환경과의 충돌 검사는 유지합니다. 내부 광학 TF는 URDF에서 중복 발행하지 않습니다.

## 컨테이너 장치

USB 재연결 또는 컨테이너 재시작 후 장치 마운트가 오래됐으면 카메라 runner를 중지한 상태에서 호스트에서 실행합니다.

```bash
python3 /home/ggh/wsyoon_data/d435-20261116/attach_d435_devices_host.py
```

기존 컨테이너의 영상 장치 목록은 생성 시점에 고정되어 있으므로 모델 선택만으로 USB 장치 전달까지 자동 갱신되지는 않습니다.

## 검증

실물 로봇과 분리된 ROS domain 183에서 D435 드라이버의 파지용 토픽을 확인했습니다. RGB 8개, 정렬 깊이 9개, CameraInfo 9개, 점군 3개를 수신했고, 같은 타임스탬프·424x240 크기의 RGB/깊이/CameraInfo 세트를 확인했습니다. 별도 robot_state_publisher와 가상 관절값만으로 base_link → d435_color_optical_frame TF 조회를 확인했습니다. CAN 드라이버·BT 시작 서비스·팔 동작은 실행하지 않았고, 시험 프로세스는 종료했습니다. 실제 물체 위치 정확도와 파지 성공은 실측 보정 및 실물 시험이 필요합니다.


## runner에서 장착 TF 수정

```yaml
d435:
  # 기존 시리얼·영상 프로파일 설정 아래에 위치합니다.
  mount:
    parent_frame: link6
    translation_m: [-0.06, 0.0, 0.065]
    rotation_rpy_deg: [180.0, -90.0, 0.0]
```

위치는 부모 링크 기준 X/Y/Z(m), 회전은 roll/pitch/yaw(도)입니다. runner가 도를 라디안으로 변환해 `piper_moveit`과 `bt_demo`에 같은 launch 인자를 전달합니다. robot_state_publisher의 장착 TF, MoveIt 충돌 형상, BT RViz의 카메라 모델이 같은 변환을 사용합니다. 센서 내부 광학 TF는 드라이버의 실제 보정값을 계속 사용합니다.

부모는 로봇 URDF에 실제 존재하는 링크여야 합니다. 존재하지 않는 링크, 자기 자신인 d435_link, 잘못된 배열 길이, NaN/무한대 등은 실행 전 거부합니다. link6 이외 부모를 지정하면 SRDF의 기존 손목·그리퍼 고정 부품 충돌 예외를 그대로 재사용하지 않고 해당 부모와 카메라 사이에만 예외를 적용합니다.

수정 후 진행 중인 작업을 마치고 `bt_demo`, `piper_moveit`을 중지한 다음 `piper_moveit` → `bt_demo` 순으로 다시 활성화합니다. runner는 매 활성화 때 YAML을 읽으므로 값만 수정했을 때 runner 프로그램 전체를 재시작할 필요는 없습니다. 실행 중 TF를 즉시 변경하지 않습니다. 현재 초기 위치·방향은 변경하지 않았고 실물 팔도 움직이지 않았습니다.

282개 관련 Python 테스트가 통과했습니다. 기존과 다른 좌표를 넣었을 때 runner의 실제 셸 인자, MoveIt/robot_state_publisher/RViz의 생성 URDF와 SRDF가 일치하는지 검증했습니다.

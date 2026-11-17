# D435 BT 연결·장착 설정

실물 파지 카메라는 `ros_runner/config/runners.yaml`의 `realsense.model`로 선택합니다.
현재 선택은 `d435`이며 시뮬레이션은 D405 구성을 사용합니다.

## 카메라와 프로파일

```yaml
realsense:
  model: d435
  enabled: true

d435:
  serial: ""  # 사용할 장치 번호; 빈 값은 D435 계열 자동 선택
  depth_profile: 480x270x6
  color_profile: 424x240x6
  use_rviz: true
```

위 시리얼은 예시입니다. `d435.use_rviz`는 센서 단독 RViz 설정이고,
BT 통합 RViz는 `piper.use_rviz`로 제어합니다.

`bt_demo` 활성화 시 의존 runner인 `piper_moveit`이 선택한 드라이버와 D435 손목 모델을 시작합니다.
단독 `d435` runner는 미리 켜지 않습니다. backend는 장치 중복 사용을 막기 위해 단독 runner를 중지합니다.
실물 BT 활성화 시 [티칭 종료 시작 절차](piper-hardware-moveit.md)가 팔을 움직일 수 있습니다.

## 파지 입력과 TF

BT 통합은 `d435.launch.py`의 `pipeline_topics:=true`로 다음 입력을 제공합니다.

| 데이터 | BT 공통 토픽 |
|---|---|
| RGB | `/camera/color/image_raw` |
| RGB에 정렬한 Depth | `/camera/depth/image_rect_raw` |
| 정렬 Depth CameraInfo | `/camera/depth/camera_info` |
| 색상 점군 | `/camera/depth/color/points` |

원시 Depth는 `/camera/depth/image_raw`로 분리합니다.
실물 robot_state_publisher가 부모 → `d435_link` 장착 TF를,
RealSense 드라이버가 센서 내부 optical TF를 발행합니다.
MoveIt과 BT RViz는 같은 D435 URDF/SRDF를 사용합니다.

## 장착값 조정

```yaml
d435:
  mount:
    parent_frame: link6
    translation_m: [-0.09, 0.05, 0.075]
    rotation_rpy_deg: [0.0, -90.0, 0.0]
```

위 값은 현재 runner의 조정값이며 실측 hand-eye 보정 완료를 뜻하지 않습니다.
위치는 부모 링크에서 SDK 깊이 원점 `d435_link`까지의 XYZ(m), 회전은 RPY(도)입니다.
runner 설정은 직접 launch의 `rwh_beyond_bringup/config/d435_mount.yaml`보다 우선합니다.
동일한 변환이 robot_state_publisher·MoveIt 충돌 형상·BT RViz에 전달됩니다.

부모는 실제 URDF 링크여야 합니다. 자기 자신·센서 자식 frame, 잘못된 배열·비유한 값은 거부합니다.
`link6` 이외 부모를 지정하면 카메라와 새 부모 사이에만 장착 충돌 예외를 적용합니다.
센서 내부 optical TF를 별도로 중복 발행하지 않습니다.

수정 후 `bt_demo`, `piper_moveit`을 중지하고 `piper_moveit` → `bt_demo` 순으로 재활성화합니다.
값만 바꿀 때 runner 전체 재시작이나 재빌드는 필요하지 않습니다.
USB 장치 문제는 [단독 점검](d435-standalone.md#usb-container-devices)을 참고하세요.
실제 위치 정확도는 분할 점군과 알려진 물체 위치를 비교해 확인합니다.

[장치 설정](runner-device-settings.md) · [좌표 변환](project-data.md)

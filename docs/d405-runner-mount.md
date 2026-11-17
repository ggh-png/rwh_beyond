# D405 장착 TF를 runner에서 설정

`ros_runner/config/runners.yaml`에서 D405를 선택하고 `d405.mount`를 수정합니다.

```yaml
realsense:
  model: d405
  # enabled, serial, profile 등 기존 설정은 유지합니다.

d405:
  mount:
    parent_frame: link6
    translation_m: [-0.06, 0.0, 0.065]
    rotation_rpy_deg: [180.0, -90.0, 0.0]
```

현재 실물 선택은 D435입니다. 위 `model: d405`는 D405로 전환하는 예시이며 장착값은 명목값입니다.

`parent_frame`은 실제 로봇 URDF의 링크 이름입니다. `translation_m`은 부모 링크에서 D405 SDK 깊이 원점인 `d405_link`까지의 X/Y/Z(m), `rotation_rpy_deg`는 같은 변환의 roll/pitch/yaw(도)입니다. 내부에서 라디안으로 변환합니다.

같은 값이 실물 `piper_moveit`의 MoveIt 및 robot_state_publisher, `bt_demo`의 RViz, 단독 `d405` 센서의 정적 장착 TF에 전달됩니다. robot_state_publisher가 장착 TF를 소유하면 센서는 해당 TF를 중복 발행하지 않습니다. 센서 내부 광학 TF는 RealSense 드라이버가 발행합니다. 시뮬레이션 설정은 유지합니다.

부모를 link6 이외로 바꾸면 기존 손목·그리퍼 고정 부품의 충돌 예외를 그대로 재사용하지 않고 카메라와 해당 부모 사이에만 예외를 적용합니다. 알 수 없는 부모 링크, 자기 자신이나 카메라 자식 프레임, 잘못된 배열, 무한대·NaN 값은 로봇 모델 생성 전에 거부합니다.

수정 후 진행 중인 작업을 마치고 `bt_demo`와 `piper_moveit`을 중지한 뒤 `piper_moveit` → `bt_demo` 순으로 다시 활성화합니다. 단독 센서는 `d405`를 재활성화합니다. runner는 활성화 시 YAML을 다시 읽습니다.

runner 설정이 없는 직접 launch는 D405 기본 모델 설정을 사용합니다.
실물 TF의 실제 위치 정확도는 장착 측정·보정으로 확인합니다.

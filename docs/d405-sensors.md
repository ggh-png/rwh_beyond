# D405 센서·장착 TF

## 공통 RGB-D 토픽

| 토픽 | 내용 |
|---|---|
| `/camera/color/image_raw` | RGB |
| `/camera/depth/image_rect_raw` | RGB 기준 정렬 Depth |
| `/camera/depth/camera_info` | 정렬 Depth에 대응하는 intrinsics |
| `/camera/depth/color/points` | 깊이 점군 |
| `/camera/depth/planning_points` | 실물 대상 영역 필터의 계획용 점군 |
| `/camera/depth/filtered_points` | MoveIt 처리 후 점군 갱신 |

coordinator는 정렬 RGB·Depth·CameraInfo와 촬영 시각 TF를 요구합니다.
유효한 frame·timestamp를 가진 빈 filtered cloud도 갱신 결과로 처리합니다.

## 장착 설정

원본은 `moma-mj-lab/config/cameras/d405.yaml`입니다.

| 설정 | 현재 명목값 |
|---|---|
| parent frame | `link6` |
| body frame | `d405_link` |
| translation | `[-0.06, 0.0, 0.065]` m |
| body RPY | `[pi, -pi/2, 0]` rad |
| calibrated | false |

렌즈는 그리퍼가 뻗는 link6 +Z 방향입니다. STL visual과 collision box는 공통 장착 설정을 사용합니다.
현재 값은 실측 hand-eye 보정 결과가 아닙니다.

| 모드 | link6 → d405_link | body → optical |
|---|---|---|
| sim | robot_state_publisher | robot_state_publisher 명목 extrinsics |
| real | robot_state_publisher | RealSense SDK 장치 extrinsics |

실물 URDF는 내부 optical TF를 중복 발행하지 않습니다.
로봇 모델 없이 센서만 실행할 때만 `publish_d405_mount_tf:=true`로 mount TF를 별도 발행합니다.

## 센서 실행

실물 `piper_moveit`은 기본적으로 D405를 포함합니다. 별도 카메라 driver와 중복 실행하지 않습니다.
센서만 실행할 때는 다음을 사용합니다.

```bash
ros2 launch rwh_beyond_bringup sensor.launch.py \
  mode:=real enable_lidar:=false enable_d405:=true
```

`d405_serial`로 장치를 고르고 `d405_profile`로 해상도/FPS를 지정합니다. 기본은 `640x480x15`입니다.
심 mode에서는 물리 카메라 driver를 시작하지 않고 MuJoCo 영상을 사용합니다.
USB 장치와 udev 접근 권한은 실행 환경에서 준비합니다.

## 고정 장착 접촉과 보정

명목 형상에서 `d405_link`와 `gripper_base`가 약 3 mm 겹쳐 이 고정 부품 쌍을 SRDF와
MuJoCo의 접촉 제외로 등록했습니다. 카메라와 손가락·외부 장애물의 충돌 검사는 유지합니다.
장착 위치를 보정하면 MoveIt·MuJoCo 모델을 빌드하고 관련 runner를 재시작합니다.

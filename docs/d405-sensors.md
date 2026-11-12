# D405 장착 모델·센서 선택·실물/시뮬 실행

## 장착 위치와 TF 소유권

이전 `link6`의 +X 쪽 박스 모델을 반대쪽 상단 장착부로 옮겼습니다.
공식 `realsense2_description/meshes/d405.stl`을 그대로 복사하고 mm → m 스케일
`0.001`을 적용했습니다. MuJoCo와 MoveIt은 같은 시각 모델 및 충돌 박스를 사용합니다.
렌즈는 기존처럼 그리퍼가 뻗는 방향을 봅니다.

장착 설정 원본은 moma-mj-lab의 `config/cameras/d405.yaml`입니다.
현재 `link6 → d405_link`의 명목 이동은 `[-0.06, 0, 0.065] m`, 카메라 body RPY는
`[pi, -pi/2, 0]`입니다. body는 ROS 카메라 좌표계이고 optical의 +Z가 렌즈 방향입니다.
**사진을 기준으로 배치한 명목 모델이며 실측 hand-eye 보정이 아닙니다.**
`calibrated: false`를 유지합니다. 실물 장착 치수를 측정하면 이 파일의 이동과 body RPY를
수정한 뒤 moma_mujoco_ros/moma_bringup을 재빌드해야 합니다.

| 모드 | link6 → d405_link | d405_link → 내부 optical 프레임 |
|---|---|---|
| sim | robot_state_publisher | robot_state_publisher의 명목 extrinsics |
| real | robot_state_publisher | RealSense SDK의 장치 내부 extrinsics |

실물 URDF는 내부 optical joint를 생략해 SDK와 중복 발행하지 않습니다.
단독 센서 실행에서 로봇 모델이 mount TF를 발행하지 않는 경우에만
`publish_d405_mount_tf:=true`로 같은 명목 mount를 별도 발행할 수 있습니다.

## 센서 선택

`sensor.launch.py`의 기본값은 기존 SICK 호출과 호환됩니다.

| 인자 | 기본값 | 의미 |
|---|---|---|
| `mode` | real | real 또는 sim |
| `enable_lidar` | true | SICK driver·설정된 scan TF 사용 |
| `enable_d405` | false | D405 RGB-D·점군 사용 |
| `d405_serial` | 빈 문자열 | 필요 시 특정 장치 선택 |
| `d405_profile` | 640x480x15 | D405 RGB·깊이 공통 해상도/FPS |
| `publish_d405_mount_tf` | false | 로봇 모델이 mount를 발행하지 않을 때만 true |

```bash
# 실제 D405만 사용
ros2 launch rwh_beyond_bringup sensor.launch.py mode:=real enable_lidar:=false enable_d405:=true
# 실제 LiDAR만 사용: 기존 hostname/namespace/params_file/start_driver 인자 유지
ros2 launch rwh_beyond_bringup sensor.launch.py mode:=real enable_lidar:=true enable_d405:=false
# 두 센서 모두 선택
ros2 launch rwh_beyond_bringup sensor.launch.py mode:=real enable_lidar:=true enable_d405:=true
```

`mode:=sim`에서는 물리 driver와 센서 static TF를 시작하지 않습니다. MuJoCo가 영상·깊이·
점군을, 시뮬 robot_state_publisher가 센서 TF를 발행합니다.
현재 `namespace`는 기존 SICK namespace를 제어하며 D405의 공통 영상 토픽은 `/camera`입니다.

## 실물 PiPER 실행 시 D405 자동 시작

기존 `piper_moveit` runner를 활성화하면 이제 `sensor.launch.py`도 포함해 D405가 켜집니다.
`camera_model=true`, `start_sensors=true`, `enable_d405=true`, `enable_lidar=false`가 기본입니다.
모터 자동 활성화는 기존처럼 false입니다.

```bash
source /root/ros2_ws/install/setup.bash
export RWH_PIPER_CAN_PORT=can0
ros2 launch ros_runner runner.launch.py
# 별도 터미널에서 ROS 환경 source 후
ros2 service call /ros_runner/activate_piper_moveit std_srvs/srv/Empty '{}'
```

카메라를 생략하려면 runner 시작 전에 `export RWH_ENABLE_D405=false`를 지정합니다.
이미 외부에서 센서를 실행한다면 `export RWH_PIPER_START_SENSORS=false`로 중복 기동을 막습니다.
단독 `d405` runner도 공통 sensor launch를 사용합니다. 통합 piper_moveit을 시작하면
단독 d405 runner를 종료하고 센서 소유권을 가져옵니다. piper_moveit 실행 중에 별도로
단독 d405 runner를 다시 활성화하지 마세요.

## BT launch의 모드 구분

`bt_demo.launch.py mode:=sim`은 기존 전체 시뮬 집기 BT입니다. 공통 sensor launch를
호출하지만 물리 카메라는 열지 않습니다.

`bt_demo.launch.py mode:=real`은 이제 실물 coordinator와 BT navigator도 시작합니다.
실물 runner는 `bt_demo`, 시뮬 runner는 `bt_demo_sim`입니다.
실물 시작·동작 순서와 현재 검증 범위는 [실물 BT 안내](real-bt.md)를 따르세요.
실물 MoveIt이 D405를 실행하므로 runner는 `start_sensors:=false`로 중복 기동을 피합니다.

## 공통 RGB-D 토픽

| 토픽 | 내용 |
|---|---|
| `/camera/color/image_raw` | RGB |
| `/camera/depth/image_rect_raw` | RGB 기준으로 정렬한 깊이 |
| `/camera/depth/camera_info` | 위 정렬 깊이와 일치하는 intrinsics |
| `/camera/depth/color/points` | 컬러를 사용한 깊이 점군 |
| `/camera/depth/image_raw` | 실물의 원본 비정렬 깊이 |
| `/camera/depth/raw_camera_info` | 원본 깊이 intrinsics |

실물에서는 D405의 RGB도 depth module에서 나오므로 `depth_module.color_profile`을
깊이와 동일하게 지정합니다. 불필요한 IR 스트림 두 개는 끕니다.

## 반영·검증 기록

센서/launch·빈 filtered cloud 갱신 검사 19개와 하위 검사 2개, runner 검사 8개를
통과했습니다. 실제 D405의 정렬 RGB-D·점군과 합성 관절 상태를 사용한 base_link →
optical TF 연결을 확인했습니다. 실물 CAN driver는 실행하지 않았습니다.
최종 시뮬 BT는 SUCCESS(error_code=0)를 반환했습니다. 시험용 프로세스는 종료했습니다.

새 시야에 이미 collision object로 표현한 테이블·대상만 남는 경우에도 MoveIt은
정상적으로 빈 filtered cloud를 발행합니다. coordinator가 이를 무시하던 부분을 수정해
유효한 frame과 timestamp를 가진 처리 결과로 인정합니다. 테이블 collision object와
새 Octomap 확인, IK·충돌 검사는 유지했습니다.

## 장착 회전 보정

D405 둥근 장착 구멍이 link6 -X(위쪽 장착면)를 향하도록 렌즈 축 주위로 +90도
회전했습니다. 렌즈는 link6 +Z 방향을 유지하며 body RPY는 `[pi,-pi/2,0]`,
명목 optical RPY는 `[0,0,pi/2]`입니다. STL·충돌 모델·실물 TF·시뮬 optical 모델은
공통 설정을 사용합니다. 실행 중인 실물 프로세스에는 적용하지 않았으므로
기존 runner/MoveIt·BT를 종료하고 다시 시작해야 합니다.

## 고정 그리퍼 베이스와의 장착 접촉

카메라를 그리퍼 쪽으로 30 mm 전진시킨 명목 위치 `[-0.06, 0, 0.065] m`에서는
CAD 카메라 형상과 `gripper_base`가 약 3 mm 겹칩니다. 두 링크는 모두 link6에
고정된 부품이므로 SRDF에 이 한 쌍을 `FixedMount` 접촉 예외로 등록하고,
MuJoCo에도 동일한 고정 부품 contact exclusion을 적용했습니다.
위치·렌즈 방향·카메라 collision 형상은 유지합니다. 움직이는 손가락과 외부 장애물에
대한 카메라 충돌 검사는 유지합니다. 실제 장착 오프셋은 별도 실측 보정 대상입니다.

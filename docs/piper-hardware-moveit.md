# 실물 PiPER: runner로 MoveIt 시험

새 ROS 컨테이너 `moma_mj_lab_1.1.2_hostcan` 안에서 실행합니다.
이 경로는 실제 CAN driver와 MoveIt·RViz를 실행합니다. SAM·GraspGenX·카메라는
필요하지 않으며 `bt_demo_sim`과 전체 grasp BT는 실행하지 않습니다.

## 현재 장비 검증 결과 (2026-11-11)

runner 서비스 활성화로 공식 driver·MoveIt·RViz 기동과 실제 6축·그리퍼 피드백을
확인했습니다. 펌웨어는 `S-V1.7-3`, SDK는 pyAgxArm 1.0.0
(commit `d90e234ba8e408cfa964b38aa9120b3131fc9f85`)입니다.
모터 활성화 서비스와 trajectory 실행은 호출하지 않았습니다.

**현재 실측 자세의 계획은 거부됩니다.** joint3 피드백은 약 `+0.01112 rad`
(`+0.637°`)이고 공식 모델 범위는 `[-2.96706, 0] rad`입니다.
MoveIt의 `CheckStartStateBounds`가 이를 거부하는 것을 확인했습니다.
실제 자세·영점·기구 모델의 일치를 확인해 유효한 관절 상태가 들어오는 것이 먼저입니다.
이 문제를 숨기기 위한 관절 제한 확대·피드백 clamp·검사 해제는 적용하지 않았습니다.
Plan이 유효해지기 전에는 Execute하지 마세요.

현재 컨테이너의 오래된 공식 MoveIt/description 빌드 캐시는 이전 MuJoCo 내부 경로를
가리켰습니다. 기존 build/install 결과를 `/root/ros2_ws/.piper-build-backup/`에
보관한 뒤 `/root/ros2_ws/src/agx_arm_ros`의 패키지로 재빌드했습니다.
control gate 스크립트의 실행 권한과 누락된 SDK도 반영했습니다.

## 실행

첫 번째 터미널에서 환경을 설정하고 runner를 시작합니다. 이미 켜 둔 runner는
한 번 종료하고 다시 시작해야 새 YAML과 환경변수가 반영됩니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RWH_PIPER_CAN_PORT=can0
export RWH_PIPER_AUTO_ENABLE=false
export RWH_PIPER_SPEED_PERCENT=10
ros2 launch ros_runner runner.launch.py
```

두 번째 터미널에서 같은 ROS 환경을 source하고 활성화합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /ros_runner/activate_piper_moveit std_srvs/srv/Empty '{}'
```

기본 그리퍼는 `agx_gripper`, RViz는 켜짐이며 실제 관절 피드백을
사용합니다. 실제 그리퍼가 없으면 runner 시작 전에
`export RWH_PIPER_EFFECTOR_TYPE=none`을 지정하세요.
GUI 없이 시험할 때는 `export RWH_PIPER_RVIZ=false`를 지정합니다.

## 티칭을 사용한 뒤 ROS 제어로 전환

티칭 종료를 BT에 자동 삽입하지 않았습니다. 현재 MuJoCo에는 티칭 모드나
`/exit_teach_mode` 서비스가 없습니다. 실물은 상태를 확인해 필요한 경우에만 전환합니다.

```bash
ros2 topic echo /feedback/arm_status --once
```

`ctrl_mode=1`은 CAN 제어, `2`는 티칭 모드입니다. `teach_status=2`(기록 종료)가
보이더라도 `ctrl_mode=2`이면 CAN 제어 전환을 확인한 것이 아닙니다.

**현재 펌웨어용 공식 `/exit_teach_mode`는 전체 관절의 원점 이동·모터 활성화까지
수행합니다.** 단순 플래그 변경이 아니므로 공간을 비우고 손을 뗀 상태에서 사용합니다.

```bash
# 실제 로봇을 움직일 수 있는 명령
ros2 service call /exit_teach_mode std_srvs/srv/Empty '{}'
# 실제 전환과 관절 위치 확인
ros2 topic echo /feedback/arm_status --once
ros2 topic echo /feedback/joint_states --once
```

`ctrl_mode=1`인지 확인하고, 필요할 때만 `/enable_agx_arm`을 호출한 뒤
실측 관절 범위 확인 → RViz Plan → 경로 확인 → Execute 순서로 진행합니다.
계속 티칭 모드이면 이동 명령을 반복하지 말고 상태와 driver 로그를 확인합니다.

## 먼저 상태와 계획 확인

```bash
ros2 topic echo /feedback/joint_states --once
ros2 param get /agx_arm_ctrl_single_node auto_enable
ros2 param get /agx_arm_ctrl_single_node control_enabled
ros2 param get /move_group use_sim_time
```

RViz에서 실제 로봇과 모델 자세가 일치하는지 확인합니다. MotionPlanning의
Planning Group은 `arm`입니다. 먼저 `Plan`으로 가까운 목표의 경로를 확인합니다.
MoveIt의 velocity/acceleration scaling은 처음에는 각각 0.1로 둡니다.
SDK의 speed_percent=10은 MoveIt scaling과 별도의 설정입니다.

실행 시 arm trajectory 상태에 따라 control gate가 자동으로 열리고 닫힙니다.
이 gate는 모터 활성화 기능이나 안전 인터록을 대체하지 않습니다.
`auto_enable=false`는 이미 켜진 모터를 끄라는 뜻이 아니라 자동 활성화를 요청하지 않는다는 뜻입니다.

## 사용자가 실제 이동을 시작할 때

주변 작업 공간과 경로를 확인한 뒤 명시적으로 모터를 활성화합니다.
다음 명령은 설정 확인 명령과 달리 **실물 모터를 활성화합니다**.

```bash
ros2 service call /enable_agx_arm std_srvs/srv/SetBool '{data: true}'
```

그다음 RViz에서 작은 변화의 목표를 `Plan`으로 확인하고 `Execute`합니다.
`Plan & Execute`는 확인 없이 바로 실행 단계로 이어지므로 첫 시험은 나눠서 진행합니다.
실행 전후 `/feedback/joint_states`의 실제 값으로 결과를 확인합니다.

## 종료

```bash
ros2 service call /ros_runner/deactivate_piper_moveit std_srvs/srv/Empty '{}'
```

runner 종료나 control gate 닫힘을 모터 전원 해제·비상정지로 해석하면 안 됩니다.
모터 disable은 하중을 지지한 상태에서 판단해야 하므로 종료 명령에 자동으로 넣지 않았습니다.

## 구성 파일

- `ros_runner/config/runners.yaml`: CAN 사전 검사·실행·상호 배타 처리
- `rwh_beyond_bringup/launch/piper_moveit.launch.py`: 실물용 기본값과 공식 launch include
- `rwh_beyond_bringup/piper_hardware.py`: 시뮬레이션 기준 MoveIt 파라미터·실물 전송 구성
- `agx_arm_ctrl/launch/start_single_agx_arm.launch.py`: 공식 CAN driver

기본값은 `can0`, `agx_gripper`, `auto_enable=false`, `speed_percent=10`,
실제 피드백 사용, `auto_control_gate=true`, RViz 켜짐입니다. 실물/시뮬레이터를
동시에 활성화하지 않으며, 실물 runner 활성화 시 시뮬레이션 BT·grasp bridge도 종료합니다.


## 실물·시뮬레이션 MoveIt 설정 통일

두 경로는 같은 공식 PiPER 모델·관절 제한·KDL IK·arm/gripper Action 이름을 사용합니다.
실물 launch는 공식 CAN driver를 유지하면서 계획 설정을 MuJoCo baseline에 맞춥니다.

| 항목 | 기존 실물 | 통일 후 실물 / 시뮬레이션 |
|---|---|---|
| flange → TCP | 0 | Z +0.1425 m |
| 계획 파이프라인 | OMPL·Pilz·STOMP·CHOMP 로드 | OMPL만 활성화 |
| 속도·가속도 scaling | 0.1 / 0.1 | 동일, 변경 없음 |
| IK | KDL | 동일, 변경 없음 |
| 관절·그리퍼 위치 제한 | 공식 URDF | 동일, 변경 없음 |
| controller Action | arm/gripper FollowJointTrajectory | 동일, 변경 없음 |
| self-filter padding | 0.1 m | 0.02 m |
| 점군 max_range / subsample | 5 m / 1 | 2 m / 2 |
| 점군 update_rate | 1 Hz | 2 Hz |
| 필터 점군 | filtered_cloud | /camera/depth/filtered_points |
| Octomap frame / resolution | 별도 기본값 | world / 0.01 m |
| 실행 시간 배율 / 여유 | 별도 기본값 | 3.0 / 2.0초 |
| RViz 파일 | 공식 moveit.rviz | 시뮬과 같은 depth_planning.rviz |

관절 feedback의 음수 그리퍼 폭이나 범위 밖 joint3를 수정·clamp하지 않습니다.
두 경로의 제한 자체가 이미 같았으므로, 설정 통일만으로 실제 영점·범위 오류가
해결되지는 않습니다.

다음 차이는 실제 장비에 필요합니다.

- 실물 `use_sim_time=false`, 시뮬레이션 `true`.
- 실물은 CAN driver와 공식 ros2_control command transport, 시뮬레이션은 MuJoCo Action 서버.
- 실물 피드백은 CAN에서 수신하며 합성 상태로 대체하지 않음.
- 실물의 모터·티칭·control gate는 유지. 시뮬레이터에 티칭 기능을 추가하지 않음.
- D405 장착 모델/TF는 실제 부착 위치가 확인되지 않아 실물에서는 기본 비활성화.

`RWH_PIPER_TCP_OFFSET`으로 실제 도구에 맞는 TCP를 지정할 수 있습니다.
시뮬과 정확히 같은 D405 mount가 확인된 경우에만 runner 시작 전에
`RWH_PIPER_CAMERA_MODEL=true`를 지정하세요. 이 옵션은 모델/TF를 추가할 뿐
실물 카메라 driver를 시작하거나 외부 보정을 수행하지 않습니다.
`RWH_PIPER_OCTOMAP=false`로 점군 처리를 끌 수 있지만, 이 경우 점군 장애물은 반영되지 않습니다.

설정 반영 후 runner를 재시작해야 합니다. 현재 소스에서 `piper_moveit_sim`은
실물 runner의 기존 이름으로 유지되어 있고, 실제 MuJoCo runner는 `piper_sim`입니다.
실물은 명확하게 `activate_piper_moveit`을 사용하세요.

검증은 같은 카메라 모델을 선택했을 때 실제 생성되는 MoveIt 파라미터가 clock 설정을
제외하고 일치하는지, 카메라를 제외해도 공식 관절 제한은 동일한지 확인합니다.
별도 ROS 도메인에서 CAN driver를 제외하고 합성 관절값으로 controller·RViz 초기화와
plan-only 요청을 검증합니다. 이는 실제 로봇의 이동 성공 검증이 아닙니다.


## 현재 확인한 오류를 구분하기

| 증상 | 확인된 의미 | 먼저 볼 것 |
|---|---|---|
| `can0` missing | 컨테이너 네트워크에서 CAN이 보이지 않음 | host network, 호스트 CAN UP·bitrate |
| `check_ready` GraspGenX deadline | 모델 endpoint 연결 실패 | 같은 네트워크, 모델 프로세스 health |
| joint3 `START_STATE_INVALID` | 실측 각도가 공식 위치 범위를 벗어남 | 실제 자세·영점·모델 일치 |
| gripper `-0.0006` 범위 오류 | 실측 폭이 최소 0보다 0.6 mm 작음 | 그리퍼 실제 개구·영점·피드백 |
| 그리퍼 mimic 두 개도 범위 오류 | 같은 그리퍼 값에서 파생된 두 손가락 값 | 독립 고장 3개로 해석하지 않음 |
| `/move_home` 응답은 왔지만 이동하지 않음 | Empty 응답 자체에는 성공 여부가 없음 | CAN 제어 모드·enable·실측 도달 |

현재 공식 `/move_home`은 모든 arm 관절 목표를 0으로 보내고 `motion_status`를
확인합니다. 실제 관절 오차를 대조하지 않으므로 성공 로그만으로 원점 도달을
판정할 수 없습니다. `/move_home`과 `/exit_teach_mode`는 MoveIt 충돌 계획을 거치지 않습니다.

범위 밖 상태는 캘리브레이션 실패의 확정 증거가 아닙니다. 현재 자세·기구 기준·영점·추종 오차를
구분해야 하며, 임의 자세에서 영점을 재설정하거나 제한을 넓히는 수정은 하지 않았습니다.

## D405 자동 시작과 새 장착 모델

후속 변경에서 실물 runner가 D405를 함께 시작하고 상단 장착 STL·TF를 사용하도록 구성했습니다. [D405 장착·센서 선택·모드 구분](d405-sensors.md)을 참고하세요. mount는 명목값이며 hand-eye 보정은 완료되지 않았습니다.


실물 전체 집기 BT는 별도 `bt_demo` runner와 실제 피드백 기반 transport를 사용합니다.
[실물 BT 준비·실행](real-bt.md)을 따르세요. 새 경로는 실물 실험 미검증 상태입니다.

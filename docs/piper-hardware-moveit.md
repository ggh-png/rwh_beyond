# 실물 PiPER 제어

실물 runner `piper_moveit`은 공식 CAN driver, robot_state_publisher, MoveIt, D405,
계획용 점군 필터와 선택한 실행 transport를 시작합니다.
`bt_demo` runner는 이를 의존 실행하고 실물 coordinator와 BT를 추가합니다.

## 준비

CAN을 준비하고 `ros_runner/config/runners.yaml`의 `piper`·`realsense` 설정을 확인합니다.
기본 CAN은 `can0`, 자동 활성화는 false, 실물 BT transport는 feedback입니다.
전체 항목은 [Runner 장치 설정](runner-device-settings.md)을 참고하세요.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch ros_runner runner.launch.py
```

별도 터미널에서 실행합니다.

```bash
ros2 service call /ros_runner/activate_bt_demo std_srvs/srv/Empty '{}'
ros2 topic echo /feedback/arm_status --once
ros2 topic echo /feedback/joint_states --once
```

## 모터와 제어 모드

모터 활성화와 명령 수신 gate는 별개입니다. 필요한 경우 사용자가 모터를 활성화합니다.

```bash
ros2 service call /enable_agx_arm std_srvs/srv/SetBool '{data: true}'
```

현재 driver 상태에서 `ctrl_mode=1`은 CAN 제어, `2`는 티칭 모드입니다.
티칭에서 돌아올 때 필요한 서비스는 다음과 같습니다.

```bash
ros2 service call /exit_teach_mode std_srvs/srv/Empty '{}'
```

이 서비스는 현재 driver에서 home 이동과 모터 활성화도 수행할 수 있으므로 실제 이동 공간을 확보한 뒤 호출합니다.
실물 `bt_demo` runner는 기본적으로 활성화 10초 후 이 서비스를 한 번 호출합니다.
사용 여부·지연·시간 제한은 `ros_runner/config/runners.yaml`의 `bt_demo_startup`에서 설정합니다.
전환 후 arm status와 관절 피드백을 다시 확인하고 `/run_demo`를 호출합니다.
실물 trajectory transport는 실행 중 `/control_enable` gate를 관리합니다.

## 실행 transport

실물 BT runner는 `feedback` transport를 요구합니다. `physical_trajectory`가
FollowJointTrajectory를 받아 실제 joint feedback을 확인하며 명령을 전달합니다.
직접 `piper_moveit.launch.py`를 호출할 때는 기본값이 `official`이므로
실물 BT용이면 `execution_transport:=feedback`을 명시합니다.

열기와 팔 이동은 실제 endpoint 확인을 사용합니다. 닫기는 설정한 폭을 4초간 명령한 뒤
완료하며 물체 잡힘은 판정하지 않습니다. 취소와 피드백 통신 오류는 별도로 처리합니다.

## 시작 상태 허용 각도

시작 허용 각도는 공통 `rwh_beyond_bringup/config/bt_demo.yaml`에서 관리합니다.

```yaml
grasp_coordinator:
  ros__parameters:
    planning_start_bounds_tolerance_deg: 10.0
```

단위는 도이며 기본값은 10°입니다. 유효 범위는 0 초과~180°이며 내부에서 rad로 변환합니다.
장치 runner와 BT를 중지·재활성화하면 변경값이 적용됩니다.
`home_joint_tolerance`(home 도착 판정, rad)와 `physical_position_tolerance`(동작 도착 판정, rad)는
같은 YAML의 별도 항목입니다. 시작 허용 각도를 변경해도 도착 판정값은 바뀌지 않습니다.
URDF 한계를 이 범위 안에서 벗어난 관절만 계획용 복사본에서 경계값으로 보정합니다.
실제 피드백과 URDF 한계는 변경하지 않으며 큰 초과는 거부합니다.
MoveIt 시작점 검사와 physical transport도 같은 허용량을 사용합니다.
transport는 실측 상태에서 계획 시작점까지 연결 구간을 만들어 명령 점프를 줄입니다.

계획 오류와 실행 오류를 구분해 확인합니다. 부분 Cartesian 경로는 실행하지 않습니다.
팔·카메라의 충돌과 실제 장착 TF는 [카메라 문서](d405-sensors.md)를 참고하세요.

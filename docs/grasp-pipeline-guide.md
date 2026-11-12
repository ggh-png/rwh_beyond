# 실행 가이드

명령은 ROS 컨테이너 `moma_mj_lab_1.1.2_hostcan` 안에서 실행합니다.
GPU 모델 서버와 필요한 네트워크 연결을 먼저 준비합니다.
서버 접속·컨테이너 시작·모델 health는 [외부 GPU 추론 서버 운영](inference-server-operation.md)을 따릅니다.

## 1. 환경과 runner 시작

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch ros_runner runner.launch.py
```

외부 서버 SSH 주소와 모델 포트는 `ros_runner/config/runners.yaml`의 `inference_connection`에서 관리합니다.
호스트 터널 서비스를 먼저 준비합니다. [런타임 설정](piper-runtime.md)을 참고하세요.

## 2. 실행 모드 선택

별도 터미널에서 ROS 환경을 source한 뒤 아래 중 하나를 호출합니다.

```bash
# 심: piper_sim 의존 실행 + ROS bridge + coordinator + BT + RViz
ros2 service call /ros_runner/activate_bt_demo_sim std_srvs/srv/Empty '{}'
```

```bash
# 실물: piper_moveit 의존 실행 + ROS bridge + coordinator + BT
ros2 service call /ros_runner/activate_bt_demo std_srvs/srv/Empty '{}'
```

활성화는 실행 환경을 시작하며 집기 Action 목표를 자동으로 보내지 않습니다.
실물에서는 CAN·모터·제어 모드를 [실물 제어 안내](piper-hardware-moveit.md)에 따라 준비합니다.

## 3. 준비 확인

```bash
ros2 service call /sam3/health std_srvs/srv/Trigger '{}'
ros2 service call /graspgenx/health std_srvs/srv/Trigger '{}'
ros2 action info /run_demo
ros2 topic hz /camera/color/image_raw
```

RGB·정렬 Depth·CameraInfo와 촬영 시각 TF가 필요합니다.
같은 로봇에 대한 MoveIt, 센서, coordinator를 중복 실행하지 않습니다.

## 4. 대상 지정과 실행

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{target_text: "yellow block"}' --feedback
```

`target_text`를 생략하면 공통 YAML의 기본 대상을 사용합니다.
`behavior_tree`를 생략하면 심·실물 모두 `demo.xml`을 실행합니다.
목표는 준비·home·관측 이동부터 시작합니다.

현재 기본 관측은 home에서 base Z 방향 0.10 m 상승과 home 로컬 Y축 40° 기울임입니다.
파지 후에는 같은 관측 자세를 거쳐 home으로 복귀하고 자동으로 그리퍼를 열지 않습니다.

## 5. 결과와 다음 실행

Action result의 `error_code=0`은 시퀀스 완료를 뜻합니다. 실제 물체 잡힘은 판정하지 않습니다.
실패 시 `failed_stage`, `error_source`, `error_msg`를 확인합니다.
계획용 부착이 남은 상태에서는 새 집기를 거부합니다. 물체 처리와 부착 상태 정리는 별도로 수행해야 합니다.

Action 취소는 [Action 인터페이스](bt-operation.md)를 참고하세요. ROS 프로세스 종료는 별도입니다.

```bash
ros2 service call /ros_runner/deactivate_bt_demo_sim std_srvs/srv/Empty '{}'
# 실물 BT를 사용했다면:
ros2 service call /ros_runner/deactivate_bt_demo std_srvs/srv/Empty '{}'
```

BT 종료가 MoveIt 등 의존 runner 전체의 종료를 뜻하지는 않습니다. 모두 종료하려면
`/ros_runner/deactivate_all`을 사용합니다.

## 직접 launch

MoveIt·controller·센서가 이미 준비된 환경에서 실행합니다.

```bash
ros2 launch rwh_beyond_bringup bt_demo.launch.py mode:=sim
# 실물 MoveIt이 이미 D405를 실행 중인 경우
ros2 launch rwh_beyond_bringup bt_demo.launch.py mode:=real start_sensors:=false rviz:=false
```

[공통 BT와 설정](real-bt.md) · [RViz](grasp-visualization.md)

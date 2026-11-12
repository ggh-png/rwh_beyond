# 런타임·CAN·모델 연결

## ROS 컨테이너

현재 컨테이너는 `moma_mj_lab_1.1.2_hostcan`, workspace는 `/root/ros2_ws`입니다.
host 네트워크에서 호스트의 SocketCAN 인터페이스와 네트워크 포트 공간을 사용합니다.

```bash
# 호스트에서 ROS 컨테이너 진입
podman exec -it moma_mj_lab_1.1.2_hostcan bash
```

컨테이너에는 GPU runtime, GUI를 사용할 경우 X11 연결, D405 USB 접근이 필요합니다.
CAN은 파일 장치가 아닌 네트워크 인터페이스입니다. 호스트에서 로봇과 일치하는 bitrate로 준비합니다.

```bash
ip -details link show can0
```

실물 runner는 기본 `can0`를 사용하며 인터페이스가 UP인지 확인합니다.
현재 장비의 CAN bitrate는 1 Mbps입니다.

## 모델 endpoint

공통 YAML 기본 주소는 SAM `tcp://127.0.0.1:5567`, GraspGenX `tcp://127.0.0.1:5566`입니다.
loopback 주소는 로봇 PC에서 접근 가능한 모델 또는 SSH 포워딩 endpoint를 뜻합니다.
외부 서버 주소가 자동으로 검색되는 것은 아닙니다.

runner는 `RWH_MODEL_ENV_FILE`의 파일을 읽으며 기본 경로는 `$HOME/rwh-inference/remote.env`입니다.
다음 변수를 통해 endpoint를 지정할 수 있습니다.

```bash
# runner 시작 전에 실제 구성에 맞게 지정
export RWH_GRASP_ENDPOINT=tcp://127.0.0.1:5566
export RWH_SEGMENTATION_ENDPOINT=tcp://127.0.0.1:5567
# 다른 환경 파일을 쓸 때:
export RWH_MODEL_ENV_FILE=/path/to/model.env
```

환경 파일은 지정한 값을 덮어쓸 수 있으므로 최종 파일 내용과 runner 시작 로그의 endpoint를 확인합니다.
SSH 터널을 사용한다면 해당 터널을 먼저 연결하고 포워딩 포트에 맞춥니다.
모델 가중치·GPU 프로세스는 ROS runner와 별도로 준비합니다.

```bash
ros2 service call /sam3/health std_srvs/srv/Trigger '{}'
ros2 service call /graspgenx/health std_srvs/srv/Trigger '{}'
```

`ROS_DOMAIN_ID`는 ROS DDS 통신에 사용되며 모델 ZMQ endpoint와는 별개입니다.

## 주요 runner 설정

| 변수 | 용도 |
|---|---|
| `RWH_PIPER_CAN_PORT` | 실물 CAN 인터페이스 |
| `RWH_PIPER_EXECUTION_TRANSPORT` | 실물 BT는 `feedback` 필요 |
| `RWH_PIPER_AUTO_ENABLE` | 모터 자동 활성화, 기본 false |
| `RWH_PIPER_SPEED_PERCENT` | 실물 driver 속도 설정 |
| `RWH_BT_RVIZ` | 심 BT의 RViz 활성화 |
| `RWH_PIPER_RVIZ` | 실물 MoveIt RViz 활성화 |
| `RWH_PIPER_START_SENSORS` | 실물 MoveIt의 센서 포함 여부 |
| `RWH_ENABLE_D405` | D405 사용 여부 |

runner 설정은 `ros_runner/config/runners.yaml`이며 변경 후 runner를 재시작합니다.
모델 timeout이면 endpoint·터널·GPU 서버 준비를, ROS 서버가 안 보이면 domain·RMW·노드 실행을 확인합니다.

[환경과 빌드](project-containers.md) · [실물 제어](piper-hardware-moveit.md)

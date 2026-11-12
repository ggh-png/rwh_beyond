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

기본 주소는 `rwh_beyond_bringup/config/bt_demo.yaml`에 이미 지정되어 있습니다.
runner로 실행할 때 별도로 `export`할 필요가 없습니다.

| 모델 | YAML 기본 endpoint |
|---|---|
| SAM 3 | `tcp://127.0.0.1:5567` |
| GraspGenX | `tcp://127.0.0.1:5566` |

고정 주소를 바꾸려면 공통 YAML의 각 bridge `endpoint`를 수정합니다.
환경별 주소가 필요하면 runner가 자동으로 읽는 `$HOME/rwh-inference/remote.env`에
`RWH_GRASP_ENDPOINT`, `RWH_SEGMENTATION_ENDPOINT`를 저장할 수 있습니다.
다른 환경 파일은 `RWH_MODEL_ENV_FILE`로 지정합니다.

적용 우선순위는 명시적인 launch 인자 → launch가 읽는 환경변수 → 공통 YAML입니다.
runner는 활성화 스크립트에서 환경 파일을 source하므로 그 파일이 같은 이름의 환경변수를
다시 지정하면 파일 값이 적용됩니다. 고정 YAML 주소를 사용하려면 해당 override를 비웁니다.
환경 파일 변경은 다음 runner 활성화 시 읽고, 실행 중인 bridge에는 재활성화 후 적용합니다.

loopback은 로봇 PC의 모델 또는 SSH 포워딩 endpoint를 뜻하며 외부 서버를 자동 검색하지 않습니다.
SSH 터널과 GPU 모델 프로세스는 ROS runner와 별도로 준비합니다.

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

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

## 외부 서버와 모델 endpoint

runner 사용 시 접속 설정 원본은 **`ros_runner/config/runners.yaml`의 `inference_connection`**입니다.
서버 주소를 `.sh`나 `remote.env`에 따로 지정하지 않습니다.

```yaml
inference_connection:
  mode: ssh
  host: 166.104.138.121
  user: ircgroup1
  ssh_port: 5022
  grasp_local_port: 15566
  sam_local_port: 15567
  grasp_remote_port: 5566
  sam_remote_port: 5567
```

| 항목 | 의미 |
|---|---|
| `host`, `user`, `ssh_port` | GPU 서버의 SSH 접속 정보 |
| `grasp_local_port`, `sam_local_port` | 로봇 PC에서 bridge가 접속하는 터널 포트 |
| `grasp_remote_port`, `sam_remote_port` | 외부 서버 안에서 모델이 사용하는 포트 |

호스트 SSH 서비스는 이 YAML을 읽어 터널을 구성하고, BT runner도 같은 YAML에서
`RWH_GRASP_ENDPOINT`, `RWH_SEGMENTATION_ENDPOINT`를 만듭니다.
현재 값은 각각 `tcp://127.0.0.1:15566`, `tcp://127.0.0.1:15567`입니다.
SSH가 서버의 5566/5567로 전달하므로 ROS에서는 localhost에 접속합니다.

## 주소 변경과 반영

YAML을 수정한 뒤 **호스트**에서 터널을 재시작합니다.

```bash
systemctl --user restart rwh-inference-tunnel
systemctl --user status rwh-inference-tunnel
```

실행 중인 BT와 bridge는 비활성화·재활성화해 새 주소를 읽습니다. 주소 수정만으로는 재빌드가 필요 없습니다.
이번 runner 실행 구조 업데이트를 처음 적용할 때는 runner 프로세스도 한 번 재시작합니다.
`remote.env`와 수동 export로 endpoint를 덮어쓰는 경로는 모델 runner에서 사용하지 않습니다.
직접 ROS launch를 호출하는 경우에는 launch 인자·환경변수·BT YAML 기본값을 사용할 수 있습니다.

## 호스트 SSH 인증과 서비스

SSH 서비스는 호스트의 인증 환경을 사용합니다. 인증 키나 비밀번호를 runner YAML에 넣지 않습니다.
호스트의 `inference_tunnel_host.py`가 `podman exec`로 컨테이너 안의 runner 설정을 읽고 SSH를 실행합니다.
현재 호스트 서비스는 이 helper를 호출하도록 연결되어 있습니다.

다른 장비에 설치할 때는 `ros_runner/scripts/inference_tunnel_host.py`를 호스트에 복사하고,
기존 SSH 인증 환경에서 해당 파일을 실행하는 서비스를 준비합니다.
`RWH_RUNNER_CONTAINER`는 ROS 컨테이너 이름, `RWH_RUNNER_CONFIG`는 컨테이너 안 YAML 경로를 지정합니다.
기본 컨테이너는 `moma_mj_lab_1.1.2_hostcan`, 기본 YAML은 설치된 `ros_runner/config/runners.yaml`입니다.
커스텀 `config_yaml`로 runner를 실행한다면 호스트 서비스에도 동일한 경로를 지정합니다.

모델을 로컬에서 실행할 때는 `mode: local`, local 포트를 모델의 실제 포트로 바꾸고 호스트 SSH 서비스를 중지합니다.
GPU 모델 프로세스 기동은 SSH 터널·ROS runner와 별도입니다.

```bash
ros2 service call /sam3/health std_srvs/srv/Trigger '{}'
ros2 service call /graspgenx/health std_srvs/srv/Trigger '{}'
```

`ROS_DOMAIN_ID`는 ROS DDS 통신용이며 모델 ZMQ endpoint와는 별개입니다.

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

서버 SSH 접속과 컨테이너·모델 시작/중지·로그·health는 [외부 GPU 추론 서버 운영](inference-server-operation.md)을 참고하세요.

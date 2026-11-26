# SSH 파일 추론 설정

로봇 컨테이너와 GPU 서버가 이미지·점군·추론 결과를 파일로 주고받는 설정입니다. 로봇에서 모델 포트에 접근하기 어려운 환경에서도 SSH로 서버에 접속할 수 있으면 사용할 수 있습니다.

!!! note "적용 버전"
    이 페이지는 파일 추론 기능이 빌드된 개발 환경을 기준으로 합니다. 기존 `v2.0.0`과 `legacy-real-2026-11-26`의 `mode: 1`은 기존 모델 연결로 집기를 실행합니다.

## 호출 모드

| 모드 | 동작 |
|---|---|
| `0` 또는 생략 | 기존 모델 연결로 집기 |
| `1` | 파일 교환으로 추론하고 집기 |
| `2` | 배치·놓기 |

파일 교환은 SAM 3·GraspGenX의 요청과 결과를 운반합니다. 카메라 촬영, 마스크와 깊이의 결합, 좌우 점군 융합, MoveIt 계획은 로봇에서 진행합니다.

## 1. 로봇 컨테이너 경로와 전송 설정

`rwh_beyond_bringup/config/bt_demo.yaml`의 `/**` 공통 ROS 파라미터에 설정합니다.

```yaml
/**:
  ros__parameters:
    file_transport: ssh
    file_local_root: /root/wsyoon_data/beyond_demo/com_msgs
    file_remote_root: ~/beyond_demo/com_msgs
    file_ssh_target: ""
    file_ssh_port: 0
    file_poll_seconds: 0.5
```

`file_ssh_target`이 비어 있고 포트가 `0`이면 Runner의 `inference_connection`에 설정된 SSH 대상과 포트를 사용합니다. SSH 호스트 키는 컨테이너에서 확인할 수 있어야 합니다. 기존 Runner 대상과 포트를 사용할 경우 로컬 Runner의 비밀번호 인증도 지원합니다. 별도 SSH 대상을 지정할 때는 키 또는 agent를 사용합니다.

양쪽에서 같은 공유 디렉터리를 볼 수 있으면 `file_transport: shared`를 사용하고, 로봇의 `file_local_root`와 서버 워커의 `--root`에서 그 디렉터리를 가리키도록 설정합니다.

### 호스트에 기록 보관 { #host-records }

[컨테이너 생성 가이드](docker-containers.md#persistent-data)의 기존 데이터 마운트는 호스트의 `~/wsyoon_data`를 컨테이너의 `/root/wsyoon_data`에 연결합니다. 이미 마운트된 컨테이너는 재생성할 필요가 없습니다.

| 위치 | 경로 |
|---|---|
| 호스트에서 직접 확인 | `~/beyond_demo/com_msgs` |
| 호스트의 실제 저장 위치 | `~/wsyoon_data/beyond_demo/com_msgs` |
| 로봇 컨테이너 | `/root/wsyoon_data/beyond_demo/com_msgs` |
| GPU 서버 | `~/beyond_demo/com_msgs` |

호스트에서 바로 확인하는 경로를 처음 만들 때 아래 명령을 실행합니다. 기존 `com_msgs`가 있으면 그 파일이나 디렉터리를 보존합니다.

```bash
mkdir -p "$HOME/wsyoon_data/beyond_demo/com_msgs" "$HOME/beyond_demo"
if [ ! -e "$HOME/beyond_demo/com_msgs" ] && [ ! -L "$HOME/beyond_demo/com_msgs" ]; then
  ln -s "$HOME/wsyoon_data/beyond_demo/com_msgs" "$HOME/beyond_demo/com_msgs"
fi
```

## 2. Runner 시작 점검 설정

`ros_runner/config/runners.yaml`에서 직접 모델 연결을 점검할 프로필을 지정합니다. 기본값은 `real`, `sim`, `monitor` 모두 `direct`입니다.

아래 예시는 실물의 직접 점검을 유지하면서 시뮬레이션을 파일 전용으로 시작하는 설정입니다.

```yaml
inference_startup_check:
  real: direct
  sim: disabled
  monitor: disabled
```

- `real`과 `sim`은 해당 BT 데모가 시작할 때 직접 모델 연결을 점검합니다.
- `monitor`는 백그라운드 모델 점검과 Runner의 자동 SSH 터널을 제어합니다. 기존 직접 연결도 함께 쓸 경우 `monitor: direct`로 유지할 수 있습니다.

설정 변경 뒤 사용하는 BT Runner를 다시 활성화합니다. `runtime_setup` 모니터는 설정을 주기적으로 다시 읽습니다.

## 3. GPU 서버에서 파일 워커 시작

GPU 서버에서 기존 [SAM 3·GraspGenX 컨테이너](inference-server-operation.md)를 켭니다. 파일 워커는 서버의 로컬 모델 포트를 사용합니다. 워커를 실행하는 Python 환경에 NumPy, Pillow, pyzmq, msgpack, msgpack-numpy가 필요합니다.

BEYOND 저장소 루트에서 아래 명령을 실행합니다. GraspGenX를 `5556`으로 공개한 배포라면 `--grasp-endpoint`의 포트를 바꿉니다.

```bash
PYTHONPATH=./rwh_beyond_model_utils python3 -m rwh_beyond_model_utils.file_exchange_worker \
  --root "$HOME/beyond_demo/com_msgs" \
  --sam-endpoint tcp://127.0.0.1:5567 \
  --grasp-endpoint tcp://127.0.0.1:5566
```

GPU 서버에서는 SSH 전송을 받는 계정과 파일 워커가 동일한 요청 경로를 사용해야 합니다. 초기 배포의 워커는 서버가 재시작되면 다시 실행합니다.

## 4. 호출하고 기록 확인

변경된 `rwh_beyond_msgs`, BT, Navigator와 두 ROS 브리지가 함께 빌드된 환경에서 데모를 실행합니다. 아래 명령은 MuJoCo 시뮬레이션 예시입니다.

```bash
ros2 service call /ros_runner/activate_bt_demo_sim std_srvs/srv/Empty '{}'
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "Coca-Cola can"}' --feedback
```

요청 폴더는 `20261126T051824Z-<요청ID>`처럼 **UTC 초 단위 시각과 요청 ID**를 사용합니다. 정면 집기는 `front/`, 다중 시점 재시도는 같은 요청 아래 `left/`, `right/`, `fused/`에 기록합니다.

| 파일 | 내용 |
|---|---|
| `tx_text.json` | SAM 3에 보낼 대상 설명 |
| `tx_image.png` | 촬영한 RGB 이미지 |
| `rx_mask.png` | 입력 이미지와 해상도가 같은 SAM 3 이진 마스크 |
| `tx_object.pcd` | GraspGenX 입력용 바이너리 XYZ 점군 |
| `rx_grasp_data.json` | 좌표계·촬영 시각·파지 위치·쿼터니언·점수 |

로봇은 `*.ready.json`을 마지막에 보내고, 서버는 결과 파일을 쓴 뒤 `*.result.json`을 기록합니다. 실패하거나 취소된 요청도 남기므로 두 파일을 비교해 어느 단계에서 멈췄는지 확인할 수 있습니다. 사용자 정의 BT에서 분할 또는 파지를 반복할 때는 단계별 `exchange_view` 값을 구분합니다.

실제 SAM 3·GraspGenX를 사용한 MuJoCo 캔 집기에서 멀티뷰 경로가 `SUCCEEDED`로 끝났고, 호스트에서 네 시점 폴더를 확인했습니다. 실물 집기는 이 설정으로 검증하지 않았습니다.

# ROS 실행 환경·CAN·모델 네트워크 전환

이 페이지의 저장·생성 스크립트는 **호스트의 `~/wsyoon/ros2-docker` 저장소**에 있습니다.
RWH 패키지 안의 ROS launch와 구분하세요.

이전 MuJoCo 개발 컨테이너 `moma_mj_lab_1.1.2_gui`의 `/root/ros2_ws`를
이미지로 저장하고, host 네트워크를 사용하는 별도 컨테이너에서 재사용합니다.
기존 Panda/Husky용 `start-robot-dev.sh`와 Compose는 그대로 사용할 수 있습니다.

## 현재 사용하는 환경

현재 작업 컨테이너는 **`moma_mj_lab_1.1.2_hostcan`**입니다.
`moma_mj_lab_1.1.2_gui`는 이전 환경이며 새 작업 경로에서는 사용하지 않습니다.

| 항목 | 현재 설정 |
|---|---|
| ROS workspace | `/root/ros2_ws` (컨테이너 writable layer) |
| 네트워크 | host: 호스트의 `can0`와 TCP 포트 공간 공유 |
| CAN | `can0`, 호스트에서 UP·1 Mbps 설정 |
| GPU | NVIDIA CDI, RTX 3060 확인 |
| GUI | X11·Xauthority bind mount |
| 최근 보존 이미지 | `localhost/rwh-piper-dev:moveit-sim-matched-20261111` |
| 모델 서버 | SAM `127.0.0.1:5567`, GraspGenX `127.0.0.1:5566` |

최근 이미지는 실행 중인 컨테이너에 추가한 SDK·빌드·설정을 보존한 스냅샷입니다.
실행 중인 컨테이너의 최초 기반 이미지 이름과 최근 스냅샷 이름은 다를 수 있습니다.
새 컨테이너 생성 스크립트는 sleep만 실행하고 로봇을 자동 활성화하지 않습니다.

## 왜 can0가 보이지 않았나

기존 MuJoCo 컨테이너는 rootless Podman의 `slirp4netns` 네트워크를 사용합니다.
호스트에서는 `can0`가 있어도 컨테이너에는 `lo`, `tap0`만 있습니다.
SocketCAN의 `can0`는 네트워크 인터페이스이므로 `--device /dev/can0`로 전달하지 않습니다.
새 실행 스크립트는 `--network host`를 사용합니다. 호스트에서 CAN을 설정하고,
컨테이너는 이미 올라온 인터페이스를 사용합니다.

현재 장비에서는 rootless Podman + host network로 별도 `--privileged`나
`NET_ADMIN` 없이 CAN RAW 소켓을 열 수 있음을 확인했습니다.
소켓을 여는 검사는 로봇 연결·CAN 프레임 수신·모터 동작까지 검증하지 않습니다.

## 1. 실행 중인 ROS 컨테이너를 이미지로 저장

호스트에서 실행합니다. 소스 수정·빌드가 진행되지 않는 시점에 저장하세요.

```bash
cd ~/wsyoon/ros2-docker
./scripts/snapshot-piper-dev.sh
```

- 기본 원본: `moma_mj_lab_1.1.2_gui` (최초 전환용)
- 생성 이미지: `localhost/rwh-piper-dev:snapshot-YYYYMMDD-HHMMSS`
- 마지막 이미지 설정: `build/piper-snapshot.env`
- 컨테이너·이미지 메타데이터: `build/piper-backups/<시간>/`

원본은 중지·삭제하지 않습니다. 실행 중인 프로세스는 이미지에서 재개되지 않습니다.
스냅샷은 pause 없이 writable layer를 저장하므로 실행 중인 데이터베이스 등의
일관성을 보장하는 백업 방식은 아닙니다. `/root/ros2_ws`의 소스·설치·빌드 결과와
컨테이너 안에 설치한 패키지는 보존합니다.

`/work`, X11 소켓, Xauthority 등 bind mount의 내용은 이미지에 포함되지 않습니다.
`/work`의 원본은 `~/wsyoon/moma-mj-lab`이며 별도로 보관해야 합니다.
메타데이터에는 환경변수·마운트 정보가 있으므로 Git에 올리지 않습니다.

다른 장비로 옮길 TAR 파일도 필요하면 저장된 이미지 이름으로 내보냅니다.

```bash
source build/piper-snapshot.env
podman save -o build/piper-dev.oci.tar "$SNAPSHOT_IMAGE"
# 복원할 장비에서: podman load -i piper-dev.oci.tar
```

## 2. host 네트워크 컨테이너 생성

먼저 호스트에서 확인합니다. PiPER의 통신 설정과 bitrate가 일치해야 합니다.

```bash
ip -details link show can0
```

현재 확인된 설정은 UP, 1,000,000 bit/s입니다. 이미 올라온 인터페이스를
스크립트가 재설정하거나 내리지 않습니다.

```bash
cd ~/wsyoon/ros2-docker
./start-piper-dev.sh
podman exec -it moma_mj_lab_1.1.2_hostcan bash
```

기본값으로 마지막 스냅샷을 사용합니다. 다른 이미지·이름을 지정할 수도 있습니다.

```bash
SNAPSHOT_IMAGE=localhost/rwh-piper-dev:snapshot-YYYYMMDD-HHMMSS \
CONTAINER_NAME=piper-hardware ./start-piper-dev.sh
```

GPU는 NVIDIA CDI로 연결하며 `ENABLE_GPU=0`으로 생략할 수 있습니다.
`CONTAINER_ENGINE=docker`도 지원하지만 이미지는 그 엔진에 먼저 load해야 합니다.
GUI DISPLAY는 호스트의 값을 사용합니다. GUI 인증 파일 위치가 다르면
`PIPER_XAUTHORITY`, `/work` 경로는 `PIPER_WORK_DIR`로 지정합니다.

새 컨테이너는 다음을 적용합니다.

- host network: 호스트의 `can0`와 TCP 포트 공간 공유, `-p` 매핑 없음
- 이미지 내부 `/root/ros2_ws` 사용: workspace 볼륨으로 덮어쓰지 않음
- `RWH_PIPER_CAN_PORT=can0`, `RWH_PIPER_AUTO_ENABLE=false`
- CAN RAW 소켓 bind 검사만 수행, CAN 프레임 전송 없음
- sleep만 실행: driver·runner·MuJoCo·모델 자동 실행 없음

이미 같은 이름이 있으면 중단합니다. 이후에는 `podman stop/start`로 관리합니다.
기존 ROS 컨테이너와 새 컨테이너의 writable layer는 서로 독립적이므로,
새로 수정한 코드는 해당 컨테이너에서 commit/push하거나 다시 snapshot해야 합니다.

## 3. SAM·GraspGenX 네트워크 전환

이전 두 모델 컨테이너는 `--network container:moma_mj_lab_1.1.2_gui`로 생성되어 있었습니다.
새 ROS 컨테이너의 `127.0.0.1`은 호스트이므로 기존 모델의 localhost와 달랐고,
`check_ready`에서 GraspGenX timeout이 발생했습니다.

**현재는 아래 전환을 완료했습니다.** 두 모델은 새 ROS 컨테이너와 같은 host 네트워크를
사용하며, 실제 모델 health와 ROS Action을 통한 분할·파지 후보 반환을 확인했습니다.
이미 전환된 현재 환경에서 migration 스크립트를 반복 실행할 필요는 없습니다.

다음 명령으로 현재 모델 설정을 보존해 새 네트워크로 전환합니다.
호스트에서 실행하며 두 모델 서버가 잠시 중단됩니다.

```bash
cd ~/wsyoon/ros2-docker
./scripts/migrate-piper-models.py
```

이 스크립트는 다음을 수행합니다.

- 두 컨테이너의 생성 인자·이미지·모델/캐시 볼륨·GPU 설정 보존
- 기존 컨테이너를 중지하고 `*-pre-hostcan-<시간>`으로 이름 변경해 보존
- 같은 이름의 새 모델 컨테이너를 새 ROS 컨테이너의 네트워크로 생성
- GraspGenX의 host를 `127.0.0.1`로 변경한 설정 파일을 read-only mount
- 두 실제 모델 서버 시작 (ROS driver와 BT는 시작하지 않음)

메타데이터와 GraspGenX 설정은 `build/piper-models/<시간>/`에 둡니다.
기본 대상은 `moma_mj_lab_1.1.2_hostcan`이며 `PIPER_ROS_CONTAINER`로 바꿀 수 있습니다.
동일 대상에 이미 전환된 경우 중단합니다. 자동 rollback은 하지 않으므로 실패 시
보존된 컨테이너·메타데이터를 확인하세요. 기존 컨테이너 stop/start는 네트워크 모드를
바꾸지 않습니다.

모델 시작 로그:

```bash
podman exec rwh-graspgenx-local tail -n 30 /data/logs/hostcan-startup.log
podman exec rwh-sam3-local tail -n 30 /data/logs/hostcan-startup.log
```

정상 시작 후 새 ROS 컨테이너에서 사용할 주소는 다음과 같습니다.

```text
GraspGenX: tcp://127.0.0.1:5566
SAM 3:     tcp://127.0.0.1:5567
```

두 모델 모두 loopback에 bind합니다. 기존 컨테이너는
`*-pre-hostcan-20261111-181036` 이름으로 중지·보존했습니다. 이미 전환된 컨테이너는 별도 helper의 `start`로
다시 켤 수 있지만, idle 컨테이너이므로 모델 프로세스도 따로 시작해야 합니다.
컨테이너를 삭제하고 재생성할 때는 옛 helper 기본값인 `moma_mj_lab_1.1.2_gui`를 쓰지 말고
새 네트워크 대상을 지정하세요.

## 4. 실물 runner로 넘어가기

실행 명령과 실제 검증 결과는 [실물 PiPER MoveIt 시험](piper-hardware-moveit.md)을 참고하세요.

새 컨테이너 안에서 환경과 패키지를 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 pkg prefix agx_arm_ctrl
ros2 pkg prefix rwh_beyond_bringup
```

실물용 runner는 `piper_moveit`입니다. `piper_moveit_sim`은 현재 실물 runner의 기존 이름이며, 실제 MuJoCo 경로는
`piper_sim`과 `bt_demo_sim`입니다. 실물 시험은 명확하게 `piper_moveit`을 사용하세요.
현재 grasp coordinator는 `simulation_only=false`일 때 하드웨어 실행을 거부하며,
`simulation_only=true` 경로는 MuJoCo 상태 서비스에 의존합니다. 실물 집기는 설정 변경만으로
활성화할 수 없습니다. 실물 전용 준비·접촉·부착·리프트 검증 구현과 카메라
RGB·Depth·CameraInfo·TF, `use_sim_time=false` 구성이 먼저 필요합니다. CAN 접근 성공은 그 설정의 완료를 뜻하지 않습니다.

실물 runner는 `RWH_PIPER_AUTO_ENABLE=false` 기본값을 사용합니다.
직접 `piper_moveit.launch.py`를 호출할 때도 `auto_enable:=false`를 명시하세요.
이 문서의 생성·검증 명령은 driver 실행이나 모터 활성화를 수행하지 않습니다.


## 현재 환경을 다시 저장할 때

이전 GUI 컨테이너를 다시 저장하면 새 환경의 수정이 빠집니다. 현재 작업 컨테이너를
명시하고 새 태그를 사용합니다. `SNAPSHOT_IMAGE`의 예시 태그는 원하는 고유 이름으로 바꾸세요.

```bash
cd ~/wsyoon/ros2-docker
SOURCE_CONTAINER=moma_mj_lab_1.1.2_hostcan \
SNAPSHOT_IMAGE=localhost/rwh-piper-dev:my-next-snapshot \
./scripts/snapshot-piper-dev.sh
```

기존 태그가 있으면 스크립트가 중단하며 덮어쓰지 않습니다. Git commit/push와 이미지
저장은 별개입니다. 이미지 저장만으로 소스 변경이 원격 저장소에 반영되지는 않습니다.

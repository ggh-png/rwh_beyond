# 컨테이너는 어떻게 구축했나

**이미지는 프로그램과 라이브러리의 기반, 컨테이너는 그 이미지를 실행한 환경,
모델 서버는 컨테이너 안에서 별도로 켜는 프로세스**입니다.
`docker start`가 성공해도 모델 서버가 준비됐다는 뜻은 아닙니다.

## 현재 PC의 세 환경

| 항목 | ROS·MoveIt | SAM 3 | GraspGenX |
|---|---|---|---|
| 컨테이너 | `moma_mj_lab_1.1.2_hostcan` | `rwh-sam3-local` | `rwh-graspgenx-local` |
| 최근 보존 이미지 | `localhost/rwh-piper-dev:moveit-sim-matched-20261111` | `localhost/rwh-sam3:f1647e8-cu130` | `localhost/rwh-graspgenx:44eb1d7-cu130` |
| 핵심 환경 | ROS 2 Jazzy, MoveIt, MuJoCo, RViz | Python, PyTorch, SAM 3 | Python, PyTorch, GraspGenX |
| 작업 코드 | `/root/ros2_ws/src` | `/opt/sam3`, `/opt/segmentation` | `/opt/GraspGenX`, `/opt/deploy` |
| 실행 | runner → simulation + BT launch | `python -u /opt/segmentation/server.py` | `bash /opt/deploy/scripts/start_server.sh` |
| 통신 | ROS 2 Topic/Service/Action + 모델별 bridge의 ZMQ client | ZMQ REP 5567 | ZMQ server 5566 |

이 PC의 `docker`는 Podman 호환 명령입니다. 아래 실행 안내는 `sudo` 없이 `docker`를 사용합니다.
빌드 helper의 실제 구현은 `podman`입니다. 일반 Docker daemon의 컨테이너와 목록이 자동 공유되지는 않습니다.

## ROS 컨테이너의 기반

`moma-mj-lab/docker/Dockerfile`은 기존
`localhost/wsyoon-solution:workspace-1.1.2-robot-legacy` 이미지를 확장합니다.
MoveIt perception·OpenGL/GUI 관련 시스템 패키지를 더하고 `/opt/moma-venv`에 MuJoCo 의존성을 설치합니다.
ROS workspace는 `/root/ros2_ws`에서 `colcon build --symlink-install`로 빌드합니다.

현재 작업 컨테이너는 이 기반의 이전 GUI 컨테이너를 스냅샷으로 보존한 뒤 host 네트워크로
생성했습니다. 이후 실물 SDK와 공식 패키지 재빌드·MoveIt 설정 통일을 반영했습니다.
[CAN·스냅샷·모델 네트워크 전환](piper-runtime.md)에 생성·복구 절차를 정리했습니다.

이 컨테이너는 기존 로컬 기반 이미지와 GUI 연결 설정을 사용합니다. 이 문서만으로 빈 PC에서
동일한 ROS 기반 이미지까지 새로 재현한다고 가정하지 않습니다. 해당 기반과 GUI 설정은
`moma-mj-lab/README.md`, `docker/Dockerfile`, 실행 스크립트를 함께 확인하세요.

## GraspGenX 이미지와 시작 과정

배포 소스: `rwh_beyond/deploy/graspgenx-docker/`.
호스트 운영 사본: `~/graspgenx_workspace/docker/`.

1. `Dockerfile`: CUDA 13.0.2 Ubuntu 24.04 runtime 기반에 Python venv·PyTorch 2.9.1 CUDA 13.0 설치.
2. NVlabs/GraspGenX 소스를 commit `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`로 고정하고 serve 의존성 설치.
3. `scripts/local_podman.sh start`: GPU 0과 ROS 컨테이너 네트워크를 연결해 idle 컨테이너 생성.
4. `start_server.sh`: checkpoint·gripper assets 위치를 준비하고 로그를 남기며 `run_server.py` 실행.
5. `run_server.py`: `config/server.local.yaml`을 읽고 공식 `client-server/graspgenx_server.py` 실행.

현재 `fast_inference: false`, 기본 gripper `piper_hand`, port `5566`입니다.
Dockerfile의 `EXPOSE 5556`보다 **실제 local YAML의 5566**이 현재 endpoint를 결정합니다.

## SAM 이미지와 시작 과정

배포 소스: `rwh_beyond/deploy/sam3-docker/`.
호스트 운영 사본: `~/sam3_workspace/deploy/`.

1. 이미 구축한 GraspGenX 이미지를 `FROM`으로 사용해 CUDA·PyTorch 계층을 재사용합니다.
2. facebookresearch/sam3 소스를 commit `f1647e8d0c1d280991f4d1b7d974c2704537fe66`로 고정합니다.
3. torchvision 0.24.1 CUDA 13.0과 SAM 의존성을 설치하고, 서버 어댑터를 `/opt/segmentation`에 복사합니다.
4. 승인된 `facebook/sam3` 체크포인트를 호스트 `~/sam3_workspace/models/sam3.pt`에 다운로드합니다.
5. 서버가 `/data/models/sam3.pt`를 읽어 공식 image model을 GPU에 로드합니다.

SAM 모델 접근 승인이 필요한 최초 설치 절차와 다운로드 helper는
[배포 README](https://github.com/ggh-png/rwh_beyond/tree/main/deploy/sam3-docker)를 참고하세요.
운영 중인 모델은 이미 다운로드되어 있으므로 매번 인증·다운로드하지 않습니다.

## 파일은 어디에 남나

| 호스트 경로 | 컨테이너 경로 | 보관 내용 |
|---|---|---|
| `~/sam3_workspace/models` | SAM `/data/models` | `sam3.pt` |
| `~/sam3_workspace/cache` | SAM `/data/cache` | 모델 캐시 |
| `~/sam3_workspace/logs` | SAM `/data/logs` | 서버 로그·실험 파일 |
| `~/graspgenx_workspace/models` | Grasp `/opt/GraspGenX/ext` | checkpoint·그리퍼 assets |
| `~/graspgenx_workspace/cache` | Grasp `/data/cache` | 다운로드 캐시 |
| `~/graspgenx_workspace/tensorrt_artifacts` | Grasp `/data/trt` | runtime artifacts |
| `~/graspgenx_workspace/logs` | Grasp `/data/logs` | 추론 서버 로그 |

두 모델 컨테이너는 root filesystem을 read-only로 두고 위 볼륨과 `/tmp`에 쓰도록 구성했습니다.
CPU 2개·RAM 8 GiB·GPU 0을 각각 사용합니다. PyTorch allocator fraction은 SAM 0.65,
GraspGenX 0.6으로 지정되어 있지만 GPU 전체의 독립적인 VRAM 할당 보장은 아닙니다.

**서버 Python 소스는 이미지에 COPY됩니다.** 호스트 운영 사본을 수정하는 것만으로 실행 중인
컨테이너 코드가 바뀌지 않습니다. 배포 소스 → 운영 사본 동기화 → 이미지 rebuild → 해당 모델
컨테이너 재생성 → health 확인 순서가 필요합니다. 모델·캐시 볼륨은 보존합니다.
이 페이지의 설명만으로 실행 중인 컨테이너를 자동 재생성하지는 않습니다.

## 왜 별도 컨테이너인데 localhost로 통신하나

현재 ROS 컨테이너는 `--network host`이고, 두 모델도 새 ROS 컨테이너의 네트워크로
전환되어 같은 호스트 네트워크를 사용합니다. 파일시스템·모델 프로세스는 별개입니다.

- ROS 안의 `tcp://127.0.0.1:5567`: SAM 서버.
- ROS 안의 `tcp://127.0.0.1:5566`: GraspGenX 서버.
- 현재는 호스트 localhost와 같은 공간이며 두 모델은 `127.0.0.1`에만 bind합니다.
- 이전 `slirp4netns` 구성과 달리 host 네트워크에서는 `can0`를 직접 볼 수 있습니다.
- 기존 모델 helper로 컨테이너를 **새로 만들 때**는 이전 GUI 컨테이너 이름이 아닌
  `SAM3_ROBOT_CONTAINER=moma_mj_lab_1.1.2_hostcan`,
  `GRASPGENX_ROBOT_CONTAINER=moma_mj_lab_1.1.2_hostcan`을 지정해야 합니다.
- 기존 컨테이너 stop/start는 네트워크를 변경하지 않습니다. 최초 전환은
  `ros2-docker/scripts/migrate-piper-models.py`로 완료했으며, 설정·이전 컨테이너를 보존했습니다.
- GraspGenX는 새 loopback 전용 YAML을 read-only mount합니다. 최근 실행 설정과 모델 볼륨은
  `~/wsyoon/ros2-docker/build/piper-models/`에 기록했습니다.
- 원격 모델 서버는 별도 endpoint·접근 경로가 필요하며 원격 배포 검증은 완료하지 않았습니다.

모델 GPU 통신에는 `ROS_DOMAIN_ID`가 사용되지 않습니다. 이 값은 ROS 노드 사이의 DDS 도메인을 맞춥니다.

다음: [실제로 오가는 데이터](project-data.md)

# GPU 추론 서버 운영

SAM 3와 GraspGenX는 ROS 컨테이너와 분리한 GPU 프로세스로 실행합니다.
서버에서 모델을 준비한 뒤 로봇 PC의 SSH 터널·ROS bridge를 연결합니다.
아래 Docker 명령은 **GPU 서버 호스트**에서 실행합니다.

## 1. 서버와 모델 준비

로봇 PC `ros_runner/config/runners.yaml`의 `inference_connection`에서
SSH 주소·계정·포트와 모델 포트를 확인합니다. 인증은 호스트 SSH 키/agent를 사용합니다.

| 구성 | 입력 → 출력 | 배포 소스 |
|---|---|---|
| SAM 3 | RGB·텍스트/박스 → 마스크 | `deploy/sam3-docker/` |
| GraspGenX | 객체 XYZ → 파지 자세·점수 | `deploy/graspgenx-docker/` |

GPU runtime·모델 체크포인트·그리퍼 assets를 준비합니다.
각 배포 디렉터리의 `.env.example`을 복사하고 장비의 GPU·사용자·볼륨·리소스를 지정합니다.
두 모델은 서로 다른 디렉터리에서 각각 빌드·실행합니다.

```bash
# 해당 모델의 deploy 디렉터리에서
cp .env.example .env
# .env를 실제 장비 설정으로 편집한 뒤
# 모델 파일·캐시 볼륨 경로도 준비합니다.
docker compose config
docker compose build
# SAM 디렉터리에서는:
docker compose up -d segmentation
# GraspGenX 디렉터리에서는 대신:
# docker compose up -d inference
```

[SAM 배포 소스](https://github.com/ggh-png/rwh_beyond/tree/main/deploy/sam3-docker) ·
[GraspGenX 배포 소스](https://github.com/ggh-png/rwh_beyond/tree/main/deploy/graspgenx-docker)

!!! note "모델 포트 맞추기"

    ROS 공통 YAML은 SAM 5567·GraspGenX 5566을 사용합니다.
    SAM Compose 기본 공개 포트는 5567이지만 **GraspGenX Compose 템플릿은 5556**입니다.
    템플릿을 그대로 쓰면 runner의 `grasp_remote_port`를 서버의 실제 공개 포트 5556으로 맞춥니다.
    모델 수신 포트·Docker 공개 포트·SSH 터널·bridge endpoint를 구분합니다.

컨테이너가 시작되면 모델 서버 명령이 실행됩니다. GPU 가중치 로딩 뒤 health가 성공해야 준비 완료입니다.
기존 서버가 실행 중일 때 같은 Python 서버를 다시 실행하면 포트 충돌이 발생할 수 있습니다.

## 2. 상태·로그·시작·종료

각 모델의 Compose 디렉터리에서 실행합니다. 아래는 SAM 서비스 예시이며
GraspGenX에는 `segmentation` 대신 `inference`를 사용합니다.

```bash
docker compose ps -a
docker compose logs --tail 100 segmentation
docker compose start segmentation
docker compose stop segmentation
docker compose restart segmentation
```

Compose 외부에서 생성한 컨테이너는 실제 이름으로 `docker ps -a`, `docker logs`,
`docker start/stop/restart`를 사용합니다. 실행 정책은 `docker inspect`로 확인합니다.
저장소 Compose 템플릿만으로 외부 배포의 이미지·재시작 정책을 단정하지 않습니다.

`docker compose logs -f`의 Ctrl+C는 로그 보기만 끝냅니다.
컨테이너 내부 확인은 `docker compose exec segmentation bash`를 사용하며,
셸의 `exit`는 서버를 종료하지 않습니다. GPU 사용은 서버 호스트의 `nvidia-smi`로 확인합니다.

## 3. 모델 health

서버 호스트의 각 Compose 디렉터리에서 확인합니다.

```bash
# SAM 디렉터리
docker compose exec -T segmentation python -c \
  "from wire import request; print(request('tcp://127.0.0.1:5567', {'action': 'health'}, timeout=5))"
# GraspGenX 디렉터리: 템플릿 포트는 5556
docker compose exec -T inference python /opt/deploy/scripts/health_check.py \
  --endpoint tcp://127.0.0.1:5556
```

포트를 바꾼 환경에서는 실제 **컨테이너 내부 수신 포트**를 지정합니다.
SAM health의 `backend=sam3`, `mock=False`와 GraspGenX의 정상 응답을 확인합니다.

## 4. 로봇 PC 연결

외부 GPU 서버를 쓸 때 [SSH 터널 설정](piper-runtime.md)을 먼저 구성합니다.
runner가 실행한 bridge의 최종 endpoint는 시작 로그에서 확인합니다.

ROS 컨테이너에서는 다음을 사용합니다.

```bash
ros2 service call /sam3/health std_srvs/srv/Trigger '{}'
ros2 service call /graspgenx/health std_srvs/srv/Trigger '{}'
```

서버 내부 health는 성공하지만 ROS health가 실패하면 터널·포트 매핑·bridge endpoint를 확인합니다.
모델 실행은 ROS runner 활성화와 별도입니다.

## 5. 변경 반영

| 변경 | 반영 방법 |
|---|---|
| SSH 주소·터널 포트 | runner YAML 수정 → 호스트 터널 재시작 → BT 재활성화 |
| ROS 대상·후보 수 | 공통 YAML 수정 → BT 재활성화 |
| 모델 코드가 이미지에 복사됨 | 이미지 재빌드 → 해당 컨테이너 재생성 |
| 모델 파일·서버 설정 | 볼륨/이미지의 실제 경로 확인 → 모델 재로딩 |
| Docker 공개 포트·볼륨 | Compose 설정 수정 → 컨테이너 재생성 |

`restart`는 기존 이미지와 생성 설정을 재사용합니다.
체크포인트·캐시·로그 볼륨은 컨테이너와 분리해 관리합니다.
운영 장애는 컨테이너 상태 → 모델 로그 → 서버 health → 터널 → ROS health 순서로 좁힙니다.

[빠른 시작](grasp-pipeline-guide.md) · [모델별 ROS 인터페이스](model-bridges.md)

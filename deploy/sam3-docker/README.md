# SAM 3 이미지 분할 서버

공식 SAM 3 image model과 `sam3.pt`를 사용하는 단일 worker GPU 서버입니다.
현재 입력은 이미지와 text/box 조건이며 결과는 선택된 한 물체의 마스크입니다.

## 구성

- `Dockerfile`: 고정한 SAM 3 소스와 서버 어댑터.
- `server.py`: image model 로딩과 ZeroMQ REP 처리.
- `segmentation_core.py`: 입력 검증과 인스턴스 선택.
- `compose.yaml`, `.env.example`: GPU 서버 배포 설정.
- `local_podman.sh`: 로컬 Podman 모델 관리.

체크포인트 접근 승인과 가중치 다운로드를 먼저 준비합니다.
서버는 `SAM3_CHECKPOINT`로 지정한 모델을 읽으며 기본 경로는 `/data/models/sam3.pt`입니다.
CUDA/PyTorch가 준비된 base image, GPU·리소스·사용자·workspace를 `.env`에 지정합니다.

```bash
cp .env.example .env
# 실제 장비의 base image·GPU·경로·리소스에 맞게 설정
# 파일 준비 후:
docker compose config
docker compose build
docker compose up -d segmentation
```

기본 published 포트는 서버 loopback의 5567입니다. 원격 사용 시 SSH 터널 또는 실제 접근 경로를
준비하고 ROS bridge의 endpoint를 맞춥니다. 로컬 helper를 쓸 때는
`SAM3_ROBOT_CONTAINER`를 실제 ROS 컨테이너 이름으로 지정합니다.

## Wire API

msgpack + msgpack-numpy를 사용합니다.
`action=segment`, uint8 HxWx3 RGB, text, 선택적 xyxy box, selection point, selection policy를 보냅니다.
결과는 binary mask·score·instance_count·backend·mock·elapsed입니다.
`action=health`로 모델 backend 준비를 확인합니다.

선택 정책은 `unique` 또는 명시적인 `highest_score`입니다.
selection point는 검출 마스크 중 인스턴스를 선택하며 native SAM point prompt가 아닙니다.
모델 실패 시 color/mock 결과로 자동 전환하지 않습니다.

[모델 동작](../../docs/sam3-perception.md) · [ROS API](../../docs/model-bridges.md) ·
[런타임 설정](../../docs/piper-runtime.md)

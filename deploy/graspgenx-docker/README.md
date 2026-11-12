# GraspGenX 모델 서버 배포

공식 GraspGenX pretrained 모델과 `piper_hand` assets를 사용하는 GPU 추론 서버입니다.
ROS·TF·IK·로봇 동작은 로봇 PC가 담당합니다.

## 구성

- `Dockerfile`: 고정한 공식 소스와 CUDA/PyTorch 실행 환경.
- `scripts/start_server.sh`, `run_server.py`: checkpoint·assets 확인과 공식 ZMQ 서버 시작.
- `config/server.local.yaml`: 프로젝트 로컬 구성, 포트 5566, `fast_inference=false`.
- `config/server.yaml`, `compose.yaml`: 별도 서버용 설정 템플릿. 템플릿의 포트는 5556이며 실제 배포와 bridge 주소를 함께 맞춥니다.
- `.env.example`: GPU·CPU·메모리·workspace·사용자·포트 설정.

모델 서버에 맞는 GPU runtime·드라이버와 checkpoint·assets를 준비합니다.
모델 데이터와 로그는 컨테이너 이미지와 별도 볼륨으로 관리합니다.
`assets_dir=auto`는 공식 소스 checkout의 그리퍼 assets 경로를 해석합니다.

## 서버 구성

배포 환경의 `.env`와 server YAML을 작성한 뒤 GPU 서버에서 실행합니다.

```bash
cp .env.example .env
# 실제 장비의 GPU·경로·사용자·리소스·포트로 .env와 server YAML을 작성
# 파일 준비 후:
docker compose config
docker compose build
docker compose up -d inference
```

SSH 터널을 사용할 경우 서버의 published loopback 포트를 로봇 PC로 포워딩하고
`RWH_GRASP_ENDPOINT`를 해당 로컬 포트로 지정합니다.
컨테이너 시작과 모델 준비는 구분하며 ROS `/graspgenx/health`로 준비 상태를 확인합니다.

GPU 추론 API는 객체 XYZ를 받아 파지 변환 행렬·점수를 반환합니다.
실제 팔의 도달 가능성·충돌은 coordinator와 MoveIt이 검사합니다.

[모델 동작](../../docs/graspgenx-model.md) · [ROS 연결](../../docs/model-bridges.md) ·
[런타임 설정](../../docs/piper-runtime.md) · [로컬 Podman 실행](LOCAL_PODMAN.md)

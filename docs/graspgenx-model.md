# GraspGenX 파지 후보

GraspGenX는 분리된 물체의 XYZ 점군과 그리퍼 정보를 받아 파지 위치·방향 후보와 점수를 반환합니다.
pretrained 모델과 `piper_hand` assets를 사용하며 프로젝트에서 별도 학습하지 않습니다.

## 서버 경로

배포 소스는 `deploy/graspgenx-docker/`입니다.
`start_server.sh → run_server.py → 공식 GraspGenX ZMQ server` 순서로 모델을 시작합니다.
기본 BT의 모델 요청은 `action=infer`이며 기본 포트는 5566입니다.

입력은 카메라 optical frame의 `float32 [N,3]` XYZ(m)입니다.
RGB·텍스트는 SAM에서, Depth 역투영은 로봇 PC에서 처리합니다.
GraspGenX에는 PiPER 관절 상태나 MoveIt Planning Scene을 전달하지 않습니다.
반환 자세는 입력 점군 좌표계로 해석하며 ROS에서 TCP 보정과 base 변환을 적용합니다.

## 현재 후보 설정

BT 공통 YAML `rwh_beyond_bringup/config/bt_demo.yaml`에서 조정합니다.

| 설정 | 현재 값 | 의미 |
|---|---|---|
| `num_grasps` | 1000 | 생성 요청 수 |
| `top_k` | 300 | 반환 후보 수 요청 |
| `max_candidates` | 300 | ROS 선택기 검사 상한 |
| `debug_max_candidates` | 10 | RViz 표시 상한 |

서버 설정에서 `fast_inference=false`이면 PyTorch 경로를 사용합니다.
GPU·가중치·서버 실행 환경은 별도로 준비하며 모델 주소는 bridge endpoint에서 지정합니다.

## 후보 선택

모델 점수는 실제 로봇의 도달 가능성이나 파지 성공 보장이 아닙니다.
`SelectFeasibleGrasp`가 TF/TCP 보정, IK, 충돌, 접근·리프트·사전 접근 경로를 검사합니다.
검사에서 모두 탈락하면 이동 목표를 실행하지 않습니다.

[좌표 변환](project-data.md) · [ROS API](model-bridges.md)

서버 SSH 접속과 컨테이너·모델 시작/중지·로그·health는 [외부 GPU 추론 서버 운영](inference-server-operation.md)을 참고하세요.

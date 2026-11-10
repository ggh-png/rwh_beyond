# SAM 3 객체 분할과 GraspGenX 연결

## 구조와 현재 검증 범위

색상에 의존하지 않는 분할 경로를 추가했다. GPU 모델은 같은 서버의 서로 다른
컨테이너에서 실행한다. ROS/MoveIt은 로봇 컨테이너에만 둔다.

```mermaid
flowchart LR
  A[등록된 RGB-D + CameraInfo + 대상 지정] --> B[InferGrasps RGBD Action]
  B --> C[ZMQ 5567: SAM 3 컨테이너]
  C --> D[선택 객체 마스크 + 분할 점수]
  D --> E[ROS CPU: 실측 Depth 역투영]
  E --> F[ZMQ 5566: 기존 GraspGenX 컨테이너]
  F --> G[PoseStamped 후보 + 파지 점수 + 마스크]
  G --> H[시뮬레이션 coordinator: TF/TCP/충돌 검사]
```

SAM 3는 마스크를 만들고 GraspGenX는 XYZ로 파지 후보를 만든다. 가벼운 Depth 역투영은
기존 CPU ROS Bridge를 재사용한다. Depth 전체를 GPU 서버로 전송할 필요가 없고
GraspGenX 공식 프로토콜도 유지한다. SAM 3D의 생성 형상이나 RGB에서 추정한 깊이를
사용하지 않는다. 두 GPU 모델은 하나의 Action 안에서 순차 호출한다.

공식 가중치 접근 승인과 fine-grained 토큰의 gated repo 읽기 권한을 확인한 뒤
다운로드·실제 SAM 3 추론을 완료했다. 저장한 시뮬레이션 RGB-D 320×240에서
텍스트 `red box`로 객체 1개를 분리하고 실제 GraspGenX 후보 20개를 ROS Action으로
수신했다. 합성 mask/mock 결과가 아니다. BT 집기의 검증 범위는 아래 최신 기록을 따른다.
SAM 실패 시 color_demo나 mock으로 자동 대체하지 않는다.

## 컨테이너 실행 (호스트 PC)

테스트한 로컬 이미지: `localhost/rwh-sam3:f1647e8-cu130`.
SAM upstream commit: `f1647e8d0c1d280991f4d1b7d974c2704537fe66`.
PyTorch 2.9.1+cu130과 torchvision 0.24.1+cu130으로 고정하고 기존 추론 이미지의
CUDA 계층을 재사용한다. 기존 컨테이너의 Python 환경은 변경하지 않는다.

```bash
# 운영 복사본: ~/sam3_workspace/deploy
bash ~/sam3_workspace/deploy/local_podman.sh build
bash ~/sam3_workspace/deploy/local_podman.sh start
# 승인된 계정으로 직접 로그인. 토큰은 터미널의 숨김 입력만 사용한다.
podman exec -it rwh-graspgenx-local /opt/venv/bin/huggingface-cli login
# 모델 접근 승인 후 다운로드
bash ~/sam3_workspace/deploy/local_podman.sh download
# foreground: Ctrl+C는 모델만 종료, idle 컨테이너는 계속 유지
bash ~/sam3_workspace/deploy/local_podman.sh model-start
# 다른 터미널
bash ~/sam3_workspace/deploy/local_podman.sh health
```

승인 신청: https://huggingface.co/facebook/sam3 . 로그인 성공과 모델 접근 승인은 별개다.
fine-grained 토큰은 해당 gated 모델의 읽기도 허용해야 한다. 토큰은 이미지나 레포에
넣지 않는다. 로컬 helper는 기존 개인 HF 캐시를 읽기 전용 `/auth`로 연결하고,
다운로드에만 사용한다. 체크포인트는 `~/sam3_workspace/models`, 캐시는 cache,
로그는 logs에 영속화한다. 인증정보·모델 파일은 Git에 포함하지 않는다.

CPU 2개/RAM 8 GiB/GPU 0 CDI/read-only root/no-new-privileges이며 GPU allocator fraction은
0.65다. 이는 모든 GPU 할당의 전역 quota가 아니다. GraspGenX·RViz와 함께 실행할 때
실측 VRAM 확인이 필요하다. 컨테이너는 기존 로봇 컨테이너 network namespace를 공유한다.
호스트 포트는 공개하지 않는다. 로봇 컨테이너 네트워크가 재생성되면 재연결이 필요하다.

원격 Docker용 compose는 localhost 포트만 공개하고 명시적 자원/GPU 할당을 요구한다.
BASE_IMAGE는 공식 PyTorch/CUDA 기반으로 필요한 Python/추론 의존성이 준비된 이미지를
지정해야 하며, 기본 로컬 GraspGenX 이미지가 원격 서버에도 있다고 가정하지 않는다.
원격 서버에는 아직 배포하지 않았다.

## ROS Action 입력·출력

기존 POINT_CLOUD=0, DEPTH_IMAGE=1에 RGBD=2를 추가했다. Action 타입 변경이므로
관련 패키지를 rebuild하고 기존 Action 서버·클라이언트/BT runner를 재시작해야 한다.

| 입력 | 의미 |
|---|---|
| rgb_image, depth_image, camera_info | 등록된 RGB-D와 그에 맞는 pinhole calibration |
| target_text | 예: box, mug. 색상과 무관한 텍스트 대상 지정 |
| target_box | 원본 이미지 pixel xyxy; 4개가 모두 0이면 미지정 |
| use_selection_point, selection_u/v | 텍스트/박스로 얻은 객체 중 해당 픽셀을 포함하는 대상 선택 |
| selection_policy | 기본 unique: 다중 후보를 거부. highest_score는 명시적 선택일 때만 사용 |
| gripper_name, num_grasps, top_k | 기존 GraspGenX 설정 |

클릭은 현재 객체 선택용이며 SAM의 직접 point prompt 입력이 아니다. 분할에는 텍스트
또는 박스가 필요하다. SAM API에는 pixel xyxy를 정규화된 cxcywh로 바꿔 전달한다.

결과에는 기존 candidates와 함께 object_mask, segmentation_score,
segmentation_instances가 들어간다. 분할 점수와 파지 점수는 다른 의미다.
마스크는 경계 배경 Depth 혼입을 줄이기 위해 3×3 침식 1회를 적용한다.
빈 마스크/잘못된 frame·timestamp·크기/비정상 모델 응답은 실패 처리한다.
Mock은 테스트에서만 명시적으로 허용하며 결과는 MOCK_ONLY로 표시한다.

```bash
# 로봇 컨테이너: 추론만, 이동 없음
source /root/ros2_ws/install/setup.bash
ros2 launch rwh_beyond_grasp_bridge inference.launch.py \
  endpoint:=tcp://127.0.0.1:5566 segmentation_endpoint:=tcp://127.0.0.1:5567

# 다른 터미널: 등록된 RGB uint8 HxWx3 NPY와 Depth NPY 입력
ros2 run rwh_beyond_grasp_bridge infer_rgbd depth.npy rgb.npy \
  --frame camera_color_optical_frame --fx 600 --fy 600 --cx 320 --cy 240 \
  --text box --box 100 100 300 300 --save-mask selected_mask.npy
```

위 calibration 숫자는 예시이며 실제 CameraInfo 값으로 바꿔야 한다.
저장 이미지 CLI는 하나의 snapshot에 동일 stamp를 부여하므로 TF/이동 검증용이 아니다.
실시간 coordinator는 촬영 시각의 TF를 추론 전에 저장한다. scene과 카메라는 추론 중
정지해 있어야 하며, 객체 이동/온라인 추적과 실물 자동 실행은 아직 지원하지 않는다.

## runner·BT와 충돌 모델

runner 기본값은 SAM 3다. `RWH_SEGMENTATION_ENDPOINT`(기본 5567),
`RWH_GRASP_ENDPOINT`(5566), `RWH_GRASP_TARGET_TEXT`(기본 red box)로 지정한다.
박스/선택 픽셀은 grasp_coordinator의 target_box/selection_point 파라미터로 지정한다.
기존 색상 demo를 의도적으로 실행할 때만 `RWH_SEGMENTATION_BACKEND=color_demo`를 사용한다.
runner는 GPU 모델 프로세스를 자동 시작하지 않는다. 기존 GraspStep prepare를 재사용하며
check_ready에서 실제 SAM 서버도 확인한다.

SAM 경로에서는 고정 4×4×5 cm 크기를 쓰지 않는다. 선택된 표면과 주변 Depth에서
가까운 수평 지지면을 추정하고, 지지면 위의 관측 AABB에 padding을 더해 충돌 객체를 만든다.
이는 보이지 않는 뒷면을 복원한 정확한 물체 모델이 아니다. 수평 테이블/작은 물체가
초기 가정이며 지지면이 불확실하면 재관측하도록 실패한다. 밀집된 물체의 완전한
충돌 형상과 일반적인 기울어진 지지면은 추가 검증이 필요하다.

MuJoCo의 `simulation/set_grasp_region` 서비스에는 선택 마스크에서 유도한 base_link
AABB를 전달한다. 이후 카메라가 움직여도 고정 base 영역으로 해당 점을 Octomap 입력에서
제외한다. 테이블을 보존하기 위해 지지면보다 3 mm 높은 곳부터 제외한다. 부피 안에
다른 물체가 겹치면 함께 제외될 수 있으므로 밀집 환경의 실물 안전성 검증을 대신하지 않는다.
실패·취소 시 영역을 복원하며, color_demo만 기존 빨간색 제외 서비스를 사용한다.

coordinator의 실제 이동·attach/lift 검증은 계속 MuJoCo 전용이다. 현재 물리 검증 서비스는
기존 시뮬레이터의 grasp_object fixture를 검사한다. 범용 실물 파지 성공 검증기는 아니다.
실제 RGBD Action은 검증했으며 BT 집기는 독립 ROS domain에서 검증한다.
기존 사용자 demo.xml을 덮어쓰지 않고 관측 자세를 포함한 baseline XML을 별도 생성한다.

## 가중치 승인 전 검증 기록 — 2026-11-10

- 메시지/Bridge/시뮬레이터/BT/navigator 5개 패키지 build 성공.
- SAM 전용 이미지 build, idle 컨테이너 기동, 공식 SAM import 및 CUDA 인식 성공.
- Bridge/runner: 49 passed, 3 subtests passed. 합성 마스크에 의한 RGBD Action 왕복,
  마스크 shape·RGB/Depth 정합·분할 서버 timeout·미검출·mock 기본 거부를 포함한다.
- 서버 순수 분할/선택 로직: 3 passed. 다중 객체 거부, 선택 픽셀, 명시적 점수 선택 검사.
- MuJoCo venv: 4 passed. 기존 닫기 유지와 선택 영역 유효성/해제 검사.
- BT/navigator test-result 집계: 28/7, errors/failures 0.
- strict MkDocs build 및 세 레포 git diff --check 성공.
- 사용자 직접 HF 로그인 성공 후 다운로드 재시도는 GatedRepoError 403으로 실패했다.
  실제 SAM 가중치 추론·마스크 정확도·통합 BT 집기는 검증하지 않았다.
- 실제 로봇 이동 명령은 전송하지 않았으며 신규 모델 컨테이너는 idle 상태로 유지한다.
  기존 GPU 서버 권한이나 호스트 패키지·드라이버·런타임은 변경하지 않았다.

```bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 python3 -m pytest -q \
  src/rwh_beyond/rwh_beyond_grasp_bridge/test src/ros_runner/test/test_piper_runners.py
cd src/rwh_beyond/deploy/sam3-docker
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_segmentation_core.py
cd /root/ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=osmesa /opt/moma-venv/bin/python -m pytest -q \
  src/moma-mj-lab/ros2_ws/src/moma_mujoco_ros/test/test_grip_hold.py \
  src/moma-mj-lab/ros2_ws/src/moma_mujoco_ros/test/test_grasp_region.py
```


## 가중치 승인 후 실제 모델 검증 — 2026-11-10

- 승인 계정으로 재로그인하고 fine-grained 토큰의 public gated repo 읽기 권한을 확인했다.
  토큰 값은 코드/문서에 기록하지 않았다. Git credential helper 설정도 변경하지 않았다.
- 공식 facebook/sam3 체크포인트 revision:
  `3c879f39826c281e95690f02c7821c4de09afae7`. sam3.pt 약 3.3 GiB를 개인 영속 경로에 다운로드했다.
- SAM 3 실제 모델 health: status=ok, backend=sam3, mock=false.
- 입력: MuJoCo 카메라가 렌더링한 등록 RGB-D 320×240. 색상 임계값/정답 mask를 사용하지 않았다.
- `box` 프롬프트는 미검출로 정상 실패했다. `red box`, `cube`, `red cube`와 수동 box prompt는
  각각 실제 마스크를 반환했다. 요청한 대상 이름/외형에 따라 검출 여부는 달라진다.
- `red box` RGBD Action: 객체 1개, 분할 점수 0.8633, 침식 후 2783 pixel,
  실제 GraspGenX 후보 20개, 입력 frame 보존, 약 1.06초(이미 로드된 모델, 단일 관측).
- 관측 GPU 메모리: SAM 프로세스 약 5480 MiB, GraspGenX 약 1002 MiB.
  이는 관측값이며 정확한 peak/일반 성능 수치가 아니다. 모델 동시 상주 성공을 확인했다.
- SAM 컨테이너 8 GiB 메모리 한도에서 모델 로딩 중 reclaim이 발생했으나
  cgroup oom/oom_kill은 0이었다. 다른 환경의 로딩 여유는 별도 측정이 필요하다.
- TF가 촬영 프레임보다 수 ms 늦게 도착하는 경우를 관측해, coordinator는 정확한 촬영
  timestamp를 유지한 채 최대 2초 동안 TF를 기다린다. latest TF로 대체하지 않는다.
- 실물 로봇 이동이나 원격 공용 GPU 서버 변경은 수행하지 않았다.

재현용 `test/sam3_action_integration.py`는 rgb/depth/k 배열이 들어 있는 NPZ snapshot을
받아 실제 두 모델의 Action 경로를 검사한다. 가중치와 두 모델 서버가 준비되어야 한다.

```bash
source /root/ros2_ws/install/setup.bash
# 다른 터미널에서 같은 domain의 inference.launch.py를 실행해 둔다.
ROS_DOMAIN_ID=92 python3 \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_grasp_bridge/test/sam3_action_integration.py \
  /tmp/sam3-observation.npz --target-text 'red box'
```

운영 아티팩트: 호스트 `~/sam3_workspace/logs/action-validation.json`,
`observation.png`, `mask-overlay.png`. 마스크 overlay는 시각적 점검용이며 정답 mask와
비교한 정확도 벤치마크를 의미하지 않는다.


### 실제 BT baseline 집기

독립 ROS domain 98에서 SAM 3와 GraspGenX를 사용한 전체 BT 집기에 성공했다.
관측 자세를 활성화하고 마지막 MoveHome을 생략한 임시 baseline 트리를 사용했다.
사용자가 수정한 원본 demo.xml은 그대로 유지한다. 따라서 홈에서 직접 인식하거나
마지막 홈 복귀를 포함한 원본 사용자 트리까지 성공했다고 해석하지 않는다.

- RunDemo status=SUCCEEDED, error_code=0.
- 물체 중심 높이 약 2.59 cm → 9.81 cm (약 7.22 cm 상승).
- 리프트의 6 cm 초과 상승/양쪽 접촉 2초 조건과 이후 2초 시점의 접촉 확인.
- 최종 양쪽 손가락 힘 각각 약 10 N. MuJoCo weld나 강제 부착 없음.
- 고정 상자 치수가 아닌 관측 점군·지지면 AABB와 선택 영역 제외 경로를 사용했다.
- 초기 검증에서 촬영 timestamp TF race를 발견하고 정확한 timestamp의 TF 대기를 추가했다.
  수정 후 baseline에서 실제 파지·리프트를 통과했다.

```bash
ROS_DOMAIN_ID=98 MUJOCO_GL=osmesa python3 \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_grasp_bridge/test/sam_pick_integration.py
```

[측정 결과 JSON](validation/sam3-local-2026-11-10.json)

관측 이미지와 SAM의 depth-safe 마스크 overlay:

![시뮬레이션 RGB 관측](assets/images/sam3-observation.png)
![SAM 마스크 overlay](assets/images/sam3-mask-overlay.png)

표면이 단순한 시뮬레이션의 단일 관측이며, 실물·복수 물체·조명 변화의 성공률은
검증하지 않았다. 미검출 시 프롬프트/관측을 바꾸며 자동 색상 fallback은 사용하지 않는다.


### runner 취소·재실행 검증

독립 ROS domain 99의 실제 runner 경로에서 다음을 확인했다.

- bt_demo → grasp_bridge → piper_sim 자동 기동.
- 후보 선택 중 취소: 선택 영역 제외 해제, 대상 CollisionObject/손가락 ACM 복원.
- 재실행: SAM 3 실제 mask와 GraspGenX 후보로 약 7.4 cm 상승, 양쪽 실제 접촉 유지.
- 이미 잡은 상태의 추가 goal은 이동/열기 전에 거부.
- runner 비활성화 후 navigator·Bridge·미사용 시뮬레이터 의존성 종료.

```bash
ROS_DOMAIN_ID=99 MUJOCO_GL=osmesa python3 \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_grasp_bridge/test/sam_runner_pick_integration.py
```

이 검증도 관측 자세 활성화/리프트까지만 실행하는 임시 baseline을 사용한다.
기본 demo.xml의 사용자 수정(관측 자세 생략/마지막 홈 복귀)은 보존했으며 그 조합의
성공 여부는 별도 검증해야 한다. 전체 SAM·GraspGenX 서버는 계속 실행 중이고,
검증용 ROS domain의 프로세스는 종료했다. 실제 상태는 health로 확인한다.

관련 메시지 변경 이후 기존 runner가 계속 실행 중이었다면 runner를 재시작해
새 설정을 로드한다. GPU 서버는 runner와 별도로 유지하며 중복 모델 기동은 피한다.

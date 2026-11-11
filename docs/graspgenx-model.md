# GraspGenX를 어떻게 활용했나

GraspGenX는 **분리된 한 물체의 3D 점군과 그리퍼 정보를 받아, 파지 위치·방향 후보와 점수를 반환**합니다.
공식 pretrained 모델과 `piper_hand` assets를 사용하며 이 프로젝트에서 모델을 별도 학습하지 않습니다.
텍스트로 물체를 찾는 일은 SAM, Depth를 XYZ로 바꾸는 일은 ROS가 먼저 수행합니다.

## 서버 실행부터 요청 처리까지

아래 경로는 GraspGenX 컨테이너 내부입니다.

```text
/opt/deploy/scripts/start_server.sh
  → /opt/deploy/scripts/run_server.py
      /opt/deploy/config/server.local.yaml 읽기
      checkpoint/gen, checkpoint/dis, piper_hand assets 존재 확인
  → /opt/GraspGenX/client-server/graspgenx_server.py
  → graspgenx/serving/zmq_server.py::GraspGenXZMQServer
      action="infer" → _handle_infer()
      _get_sampler("piper_hand")
      graspgenx/grasp_server.py::GraspGenXSampler.run_inference()
      grasps / confidences / gripper_name / timing 반환
```

SAM 서버는 프로젝트에서 만든 작은 어댑터이고, GraspGenX는 공식 ZMQ server를 실행하는
시작 스크립트를 프로젝트에서 감싼 형태입니다. 두 모델을 한 Python 프로세스에 import하지 않습니다.

## 모델 입력과 그리퍼 정보

입력 점군은 `float32 [N,3]`, 미터 단위이며 카메라 optical frame 기준입니다.
`gripper_name="piper_hand"`는 서버가 준비한 그리퍼 assets와 sampler를 선택합니다.
로봇 전체 관절값이나 MoveIt Planning Scene을 보내는 필드는 없습니다.

sampler는 점군 중심을 기준으로 전처리하고 모델의 생성·점수 평가 경로를 사용합니다.
내부 전처리 후에는 위치를 입력 점군 좌표계로 돌려 반환합니다. ROS가 별도로 물체 중심을
더하거나 빼면 좌표가 중복 보정되므로 입력 header와 촬영 TF를 유지해야 합니다.

현재 config는 `fast_inference: false`로 PyTorch 경로를 사용합니다.
TensorRT artifacts 볼륨이 존재한다고 fast runtime이 켜졌다는 뜻은 아닙니다.

## 모델의 결과를 그대로 실행하지 않는 이유

모델 점수는 **그리퍼·물체 형상을 바탕으로 한 후보 평가값**입니다.
PiPER의 관절 제한·도달 범위·현재 자세·테이블/주변 장애물·경로 실행 가능성을 모두 증명하지 않습니다.

ROS Bridge는 응답 행렬과 점수를 검증하고 `GraspCandidate[]`로 변환합니다.
coordinator의 `SelectFeasibleGrasp`가 그 후보에 TCP 보정·IK·충돌·경로 검사를 적용합니다.
따라서 후보가 여러 개 있어도 실행 가능한 후보가 없어 선택 단계가 실패할 수 있습니다.

## 현재 사용하는 API 범위

기본 BT가 보내는 ZMQ 작업은 **`action="infer"`**입니다.
현재 공식 소스에 다른 inference action이 있더라도 이 프로젝트가 자동으로 그것을 사용하는 것은 아닙니다.
전체 scene·텍스트·Depth를 GraspGenX에 보내는 현재 BT 경로는 없습니다.

ROS 쪽 `/infer_grasps`는 입력 모드 POINT_CLOUD/DEPTH_IMAGE/RGBD를 제공하지만,
후자의 전처리도 ROS/SAM에서 수행하고 GraspGenX에는 동일한 객체 XYZ 요청을 보냅니다.
구체적인 배열·필드는 [모델 데이터 형식](project-data.md)을 참고하세요.

## 바꾸는 위치

| 목적 | 위치 |
|---|---|
| 후보 생성/반환 개수 | ROS `grasp_pipeline.yaml`의 `num_grasps`, `top_k` |
| 실제 검사 후보 개수 | 같은 파일 `max_candidates` |
| RViz에만 그릴 개수 | 같은 파일 `debug_max_candidates` |
| checkpoint·port·fast runtime | 배포 `config/server.local.yaml` |
| 공식 서버 실행 옵션 | 배포 `scripts/run_server.py` |
| 로봇 실행 후보 선택 규칙 | ROS `selection_stages.py` |

공식 모델 구현을 읽고 싶으면 `grasp_server.py`와 `serving/zmq_server.py`부터,
이 프로젝트의 연결을 바꾸려면 `inference_action_server.py`와 `pose_converter.py`부터 읽으세요.

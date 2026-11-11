# SAM 3를 어떻게 활용했나

SAM 3는 이 프로젝트에서 **RGB 영상에서 지정한 한 물체의 픽셀 마스크를 만드는 역할**입니다.
공식 image model을 추론용으로 사용하며 이 프로젝트에서 별도 학습이나 fine-tuning은 하지 않습니다.
RGB에서 3D 모델을 생성하는 SAM 3D나 동영상 추적 기능은 현재 경로에서 사용하지 않습니다.

## 요청 한 번의 실제 구현

배포 코드 위치는 `rwh_beyond/deploy/sam3-docker/`입니다.

```text
ROS perception_stages.py::segment_target()
  → segmentation_client.py::segment_image()
      RGB + text/box/selection_point → ZMQ 5567
  → server.py::serve()
      segmentation_core.py::validate_request()
      Sam3.predict()
        processor.set_image(...)
        processor.set_text_prompt(...)       # text가 있을 때
        processor.add_geometric_prompt(...)  # box가 있을 때
      segmentation_core.py::select_instance()
      선택 mask + score + instance_count 반환
  → ROS에서 응답 검증·마스크 침식·Image 변환·RViz 발행
```

## 공식 모델을 로드하는 방법

`server.py::Sam3.__init__()`에서 다음 API를 사용합니다.

- `build_sam3_image_model(checkpoint_path=..., load_from_HF=False, enable_inst_interactivity=False, compile=False)`
- `Sam3Processor(model, confidence_threshold=0.5)`
- 로컬 checkpoint 기본 경로 `/data/models/sam3.pt`

서버 시작 때 한 번 모델을 로드하고 요청마다 새 이미지 state를 만듭니다.
GPU 추론은 `torch.inference_mode()`와 CUDA bfloat16 autocast 안에서 실행합니다.
서버는 단일 worker로 요청을 순차 처리합니다.

## 텍스트·박스·점의 차이

| 입력 | 실제 활용 |
|---|---|
| `text="red box"` | 공식 `set_text_prompt`에 전달하는 대상 설명 |
| pixel `box=[x0,y0,x1,y1]` | 정규화된 중심/폭/높이로 변환해 positive geometric box prompt로 전달 |
| `selection_point=[u,v]` | 모델이 검출한 mask 중 그 픽셀을 포함하는 인스턴스 선택 |
| `selection_policy="unique"` | 선택 결과가 한 개로 명확하지 않으면 오류 |
| `selection_policy="highest_score"` | 명시적으로 허용했을 때 점수 기반 선택 |

현재 어댑터는 텍스트 또는 박스를 요구합니다. **점 하나만 SAM에 직접 입력하는 interactive prompt API는
연결하지 않았습니다.** 기본 파라미터는 `target_text="red box"`, 박스·점 미지정, `unique`입니다.
선택 조건·마스크 수 검사는 `segmentation_core.py::select_instance()`가 담당합니다.

## 응답 이후 ROS에서 처리하는 것

1. `backend="sam3"`, `mock=False` 확인. 색상 기반/mocked 결과로 자동 대체하지 않음.
2. 마스크 크기·`uint8`·0/1 값·점수·인스턴스 수 확인.
3. 3×3 침식 1회로 Depth 경계 오염을 줄임. 빈 마스크면 실패.
4. ROS `Image`로 변환하고 촬영 stamp·optical frame을 붙임.
5. Depth/K와 결합해 객체 점군 생성. 이는 SAM GPU 코드가 아니라 ROS CPU 코드.

RViz의 `/grasp/object_mask_preview`는 흑백으로 보이도록 0/255로 변환한 **처리 후 마스크**입니다.
현재 발행 경로는 침식 전 SAM 원본 마스크를 별도 토픽으로 제공하지 않습니다.
`/grasp/segmentation_overlay`도 이 처리 후 마스크를 RGB 위에 합성합니다.

## 코드와 설정을 바꾸는 위치

| 바꾸고 싶은 것 | 위치 |
|---|---|
| 찾을 물체·박스·선택점 | coordinator 파라미터, [실행 중 대상 변경](grasp-pipeline-guide.md#target) |
| 모델 confidence threshold / autocast / 공식 API | 배포 `server.py::Sam3` |
| 여러 mask 중 선택 규칙 | 배포 `segmentation_core.py` |
| ROS 요청 payload·마스크 후처리 | Bridge `segmentation_client.py` |
| 촬영·TF 저장·분할 단계 흐름 | Bridge `perception_stages.py` |
| 체크포인트 경로·GPU·볼륨 | 배포 `Dockerfile`, `local_podman.sh` |

서버 코드를 바꾸면 이미지 rebuild·컨테이너 재생성이 필요하고, ROS 쪽 코드 변경과는 배포 경로가 다릅니다.
[컨테이너 구축](project-containers.md)을 먼저 확인하세요.

## 어디까지 확인됐나

실제 승인된 checkpoint와 MuJoCo RGB-D에서 `red box` 분할 및 GraspGenX 연결을 검증했습니다.
새 물체·실물·복수 물체·조명 변화의 일반적인 성공률이나 동적 추적은 검증하지 않았습니다.
초기 승인 전 테스트, 승인 후 실제 모델, 과거 `GraspStep` 방식과 실패 기록은
[이전 SAM 통합 기록](sam3-history.md)에 별도로 보존했습니다.

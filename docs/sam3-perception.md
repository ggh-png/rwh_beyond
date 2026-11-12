# SAM 3 객체 분할

SAM 3 서버는 한 RGB 이미지와 텍스트/박스 조건에서 해당 물체의 마스크 후보를 반환합니다.
ROS bridge가 선택된 한 인스턴스의 마스크를 coordinator에 전달합니다.

## 입력과 출력

- 입력: RGB, `target_text`, 선택적 pixel xyxy box, 인스턴스 선택용 pixel point.
- 출력: binary 마스크, segmentation score, instance count, backend/mock 정보.
- ZMQ 형식: msgpack + msgpack-numpy, `action=segment` 또는 `action=health`.
- 실제 서버 구현: `deploy/sam3-docker/server.py`, `segmentation_core.py`.
- ROS 구현: `rwh_beyond_sam3_bridge`.

선택 point는 검출 마스크 중 하나를 고르는 조건이며 모델의 native point prompt가 아닙니다.
SAM 서버는 로봇 TF·IK·그리퍼 제어를 수행하지 않습니다.

## 대상 지정

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{target_text: "yellow block"}' --feedback
```

요청별 텍스트가 없으면 공통 YAML의 `target_text`를 사용합니다.
짧은 물체 이름과 시각적 속성으로 대상을 표현합니다. 긴 지시문이나 관계 추론은 별도 구현이 없습니다.
예: `blue cup`, `bottle`, `crumpled tissue`. 특정 물체의 인식 성능은 실제 영상으로 확인해야 합니다.

## 인스턴스 선택

| `selection_policy` | 동작 |
|---|---|
| `unique` | 대상을 하나로 정할 수 없으면 실패 |
| `highest_score` | 명시적으로 선택한 경우 최고 점수 인스턴스 사용 |

공통 기본값은 `unique`입니다. 박스와 selection point로 대상을 좁힐 수도 있습니다.
`/run_demo`는 현재 요청별 text를 제공하며 박스·point·policy는 coordinator 설정을 사용합니다.
모델 실패 시 색상 마스크나 mock 결과로 자동 전환하지 않습니다.

## 서버와 환경

서버는 공식 image model과 준비한 `sam3.pt`를 사용합니다. 현재 어댑터는 요청마다 새 이미지 상태를 만듭니다.
서버는 단일 worker로 요청을 처리하며 기본 포트는 5567입니다.
가중치 접근 승인·다운로드와 GPU 환경은 ROS 실행 전에 준비합니다.

[모델 ROS 인터페이스](model-bridges.md) · [런타임 설정](piper-runtime.md)

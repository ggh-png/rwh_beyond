# rwh_beyond_grasp_bridge

모델 bridge와 pipeline의 Python import·CLI·launch 별칭을 제공하는 패키지입니다.
실제 기능은 다음 패키지가 소유하며 기본 BT는 해당 구현을 직접 사용합니다.

| 기능 | 구현 패키지 |
|---|---|
| SAM Action | `rwh_beyond_sam3_bridge` |
| GraspGenX Action·CLI | `rwh_beyond_graspgenx_bridge` |
| 파지 coordinator·시각화 | `rwh_beyond_grasp_pipeline` |
| 변환·wire helper | `rwh_beyond_model_utils` |

[현재 구조와 API](../docs/model-bridges.md) · [실행 가이드](../docs/grasp-pipeline-guide.md)

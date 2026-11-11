# rwh_beyond_grasp_bridge — compatibility package

현재 구현은 모델별 ROS 패키지로 이동했습니다.

| 기능 | 소유 패키지 |
|---|---|
| SAM ROS server/client | `rwh_beyond_sam3_bridge` |
| GraspGenX ROS server/CLI | `rwh_beyond_graspgenx_bridge` |
| coordinator·단계 처리·시각화 | `rwh_beyond_grasp_pipeline` |
| 공통 변환·wire helper | `rwh_beyond_model_utils` |

이 패키지는 기존 Python import·CLI·launch를 위한 호환 진입점입니다.
기존 클래스/함수를 복제하지 않고 새 소유 모듈로 위임합니다. 새 기능은 새 패키지에서 수정하세요.
기존 `ros2 launch rwh_beyond_grasp_bridge grasp_pipeline.launch.py`, inference launch와
`ros2 run rwh_beyond_grasp_bridge infer_cloud/infer_depth/infer_rgbd`는 유지합니다.
현재 기본 BT는 새 pipeline 패키지를 직접 실행합니다.

기존 회귀·통합 스크립트는 이 패키지의 `test/`에 보존합니다.
[새 구조와 API](../docs/model-bridges.md) · [실행](../docs/grasp-pipeline-guide.md)

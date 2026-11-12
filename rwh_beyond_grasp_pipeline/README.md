# rwh_beyond_grasp_pipeline

관측·로컬 점군·파지 후보 선택·MoveIt 실행·시나리오 상태·RViz 출력을 관리합니다.
GPU 모델은 별도 bridge의 ROS Action으로 호출합니다.

- 심 backend: `grasp_coordinator`.
- 실물 backend: `hardware_grasp_coordinator`.
- API: `/grasp/*`, `/grasp_step`, `/grasp/cleanup`.
- 공통 시나리오: `rwh_beyond_bt/behavior_trees/demo.xml`.
- BT 통합 설정: `rwh_beyond_bringup/config/bt_demo.yaml`.

`bt_demo.launch.py`가 두 bridge와 coordinator를 시작합니다.
직접 `grasp_pipeline.launch.py`를 사용할 때는 `mode`, `params_file`, `sam3_params_file`,
`graspgenx_params_file`과 endpoint를 지정할 수 있으며 기본값은 각 패키지의 독립 설정입니다.

[코드 안내](../docs/project-code-map.md) · [실행](../docs/grasp-pipeline-guide.md) ·
[모델 API](../docs/model-bridges.md)

# rwh_beyond_grasp_pipeline

관측·로컬 점군·파지 후보 선택·MoveIt 실행·시나리오 상태·RViz 출력을 관리합니다.
GPU 모델은 별도 bridge의 ROS Action으로 호출합니다.

- 심 backend: `grasp_coordinator`.
- 실물 backend: `hardware_grasp_coordinator`.
- API: `/grasp/*`, `/grasp/cleanup`.
- 공통 시나리오: `rwh_beyond_bt/behavior_trees/demo.xml`.
- BT 통합 설정: `rwh_beyond_bringup/config/bt_demo.yaml`.

`bt_demo.launch.py`가 두 bridge와 coordinator를 시작합니다.
직접 `grasp_pipeline.launch.py`를 사용할 때는 `mode`, `params_file`, `sam3_params_file`,
`graspgenx_params_file`과 endpoint를 지정할 수 있으며 기본값은 각 패키지의 독립 설정입니다.

[코드 안내](../docs/project-code-map.md) · [실행](../docs/grasp-pipeline-guide.md) ·
[모델 API](../docs/model-bridges.md)


## BT 인식 경로

RGB-D 촬영, SAM 요청, 마스크·깊이 점군 투영과 GraspGenX 요청은 이제
`rwh_beyond_bt`가 직접 수행합니다. 이 패키지는 인식 중계 서버를 제공하지 않습니다.
`grasp/select_feasible_grasp`는 BT가 소유한 촬영 묶음·마스크·점군·후보를 받아 검증합니다.
접근·리프트 거리도 BT 입력으로 받을 수 있으며 최종 TCP 목표를 반환합니다.
MoveIt 계획·충돌 장면·부착·실물 실행은 현재 이 모션 backend에 남아 있습니다.
[BT 포트와 현재 역할](../docs/bt-grasp-stages.md)을 참고하세요.

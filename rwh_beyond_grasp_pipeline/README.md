# rwh_beyond_grasp_pipeline

BT 파지 실행을 지원하는 장치·센서 노드와 모델 브릿지 launch입니다.

- `target_cloud_filter`: 선택한 대상 영역을 계획용 점군에서 제외합니다.
- `physical_trajectory`: PiPER의 FollowJointTrajectory 전송·취소·피드백 확인을 수행합니다.
- `feedback.py`: 실물 피드백 검증 공통 코드입니다.
- `grasp_pipeline.launch.py`: SAM·GraspGenX 브릿지를 실행합니다.

coordinator는 제거했습니다. 후보 선택·좌표 변환·계획·모션·장면·부착은
rwh_beyond_bt가 직접 수행하며 MoveIt 서비스·액션과 controller 액션을 호출합니다.
설정은 rwh_beyond_bringup/config/bt_demo.yaml의 bt_navigator.ros__parameters.bt_motion입니다.

[구조](../docs/bt-architecture.md) · [포트](../docs/bt-grasp-stages.md)

브릿지 launch 입력은 `sam3_params_file`, `graspgenx_params_file`, `segmentation_endpoint`,
`endpoint`, `use_sim_time`입니다. 대상 텍스트는 `/run_demo` 또는 BT navigator에 지정합니다.

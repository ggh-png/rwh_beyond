# 코드를 따라 읽기

| 위치 | 책임 |
|---|---|
| rwh_beyond_bringup/launch/bt_demo.launch.py | 센서·브릿지·navigator·RViz 실행 |
| rwh_beyond_bringup/config/bt_demo.yaml | bt_perception·bt_motion·MoveIt 공통 설정 |
| rwh_beyond_navigator/src/demo_navigator.cpp | RunDemo·오류·취소·Blackboard 정리 |
| rwh_beyond_navigator/src/xml_tree_filter.cpp | Groot 빈 속성 제거·include 정리·변경된 XML 재로드 |
| rwh_beyond_bt/behavior_trees | 집기 2종·배치 XML |
| rwh_beyond_bt/plugins/action/perception_actions.cpp | 촬영·SAM·점군·모델 직접 호출 |
| rwh_beyond_bt/plugins/action/motion_actions.cpp | 각 모션 BT 단계의 입력·실행·출력·취소 |
| rwh_beyond_bt/src/motion_context.cpp | TCP·폭·형상·후보 선택, MoveIt/그리퍼 클라이언트·장면·피드백 |
| rwh_beyond_grasp_pipeline | 모델 브릿지 launch, target_cloud_filter, physical_trajectory, feedback |
| rwh_beyond_sam3_bridge | SegmentImage·SAM health |
| rwh_beyond_graspgenx_bridge | InferGrasps·모델 health |
| rwh_beyond_model_utils | 영상·점군·모델 프로토콜 변환 |
| rwh_beyond_msgs | RunDemo·SegmentImage·InferGrasps·SetGraspRegion |

coordinator와 전용 Python 모션 모듈·호환 alias·중계 액션은 제거했습니다.
물리 전송과 점군 필터는 장치·센서 기능이므로 별도 노드로 유지합니다.

[구조](bt-architecture.md) · [데이터](project-data.md)

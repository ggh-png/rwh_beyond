# RViz 디버깅

coordinator가 분할·점군·파지 결과를 Reliable / Transient Local / Keep Last 1로 발행합니다.
노드가 실행 중이면 나중에 연결한 RViz에서도 마지막 결과를 받을 수 있습니다.
이 데이터는 마지막 관측의 스냅샷이며 이동 중 물체를 추적한 결과가 아닙니다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/grasp/object_mask` | Image | 처리용 binary 0/1 마스크 |
| `/grasp/object_mask_preview` | Image | 표시용 0/255 마스크 |
| `/grasp/segmentation_overlay` | Image | RGB와 선택 마스크 합성 |
| `/grasp/object_cloud` | PointCloud2 | 모델 입력용 camera frame 점군 |
| `/grasp/object_cloud_base` | PointCloud2 | 촬영 TF를 적용한 base_link 점군 |
| `/grasp/grasp_candidates` | MarkerArray | 모델 후보 |
| `/grasp/selected_grasp` | MarkerArray | 선택 TCP·pregrasp·접근 방향 |
| `/grasp/selected_tcp` | PoseStamped | 선택 TCP |
| `/grasp/selected_pregrasp` | PoseStamped | 선택 pregrasp |

`debug_max_candidates`는 표시 개수만 제한합니다. 모델 원점과 TCP는 다르므로 후보 화살표와
선택 TCP의 위치가 다를 수 있습니다. 새 관측 시 이전 표시 일부를 초기화합니다.

## 실행

심은 `bt_demo_sim`이 통합 RViz를 실행하고, 실물은 `piper_moveit`이 RViz를 실행합니다.
기본 설정은 `moma_bringup/config/depth_planning.rviz`입니다.
추가 결과 전용 뷰어는 다음과 같이 실행할 수 있습니다.

```bash
ros2 launch rwh_beyond_grasp_pipeline grasp_visualization.launch.py
```

직접 display를 추가하면 Fixed Frame은 `base_link`, 이미지에는 preview/overlay,
PointCloud2에는 `object_cloud_base`를 사용합니다. durability를 Transient Local로 맞춥니다.

## 심 전용 부분 실행

`perception_preview.xml`은 home·열기·관측 이동 후 SAM과 점군까지 실행합니다.
`grasp_debug.xml`은 후보 생성과 경로 선택까지 실행합니다.
두 트리 모두 관측 이동을 수행하며 현재 보조 트리의 이동 노드는 심 전용입니다.
실물 기본 시나리오는 `demo.xml`을 사용합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{behavior_tree: "/root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/grasp_debug.xml", target_text: "yellow block"}' \
  --feedback
```

결과가 없으면 coordinator 실행, 분할·점군 단계 성공, 동일 ROS domain과 QoS를 확인합니다.
모델 서버만 시작해서는 RViz 결과 토픽이 생성되지 않습니다.

[단계별 데이터](project-data.md) · [실행 가이드](grasp-pipeline-guide.md)

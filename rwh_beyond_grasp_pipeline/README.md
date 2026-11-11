# rwh_beyond_grasp_pipeline

관측·로컬 점군 생성·후보 선택·MoveIt 실행·공유 상태·RViz 결과를 관리합니다.
모델 통신은 각 bridge에 ROS Action/Service로 요청합니다. 직접 모델 ZMQ 요청은 하지 않습니다.

```bash
# simulation/MoveIt은 준비되어 있어야 합니다.
ros2 launch rwh_beyond_grasp_pipeline grasp_pipeline.launch.py
```

이 launch는 SAM·GraspGenX bridge와 coordinator 세 ROS 노드를 시작합니다.
BT를 쓸 때는 `bt_demo.launch.py`가 이를 포함하므로 별도 중복 실행하지 않습니다.

- 노드: `grasp_coordinator`.
- `/grasp/*` 단계별 Action, `/grasp/cleanup` 정리 서비스.
- 파라미터: `config/grasp_pipeline.yaml`.
- 모델 설정: `sam3_params_file`, `graspgenx_params_file`로 각 모델 YAML 지정.
- 모델 주소: `segmentation_endpoint`, `endpoint` launch 인자가 각 bridge로 전달됩니다.
- `grasp_visualization.launch.py`는 결과 전용 RViz입니다.

[실행](../docs/grasp-pipeline-guide.md) · [코드](../docs/project-code-map.md) · [패키지/API](../docs/model-bridges.md)

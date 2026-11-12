# rwh_beyond_graspgenx_bridge

독립 GraspGenX ROS 인터페이스입니다. 별도 GPU 모델 서버의 주소를 endpoint로 지정합니다.

```bash
ros2 launch rwh_beyond_graspgenx_bridge bridge.launch.py endpoint:=tcp://127.0.0.1:5566
```

- 노드: `graspgenx_bridge`, 실행 파일: `inference_action_server`.
- Action `/infer_grasps`: `rwh_beyond_msgs/action/InferGrasps`.
- Service `/graspgenx/health`: `std_srvs/srv/Trigger`.
- POINT_CLOUD/DEPTH_IMAGE 입력은 SAM 서버 없이 동작합니다.
- RGBD 입력은 별도 `/segment_image` Action을 요청합니다. SAM에 직접 ZMQ 요청하지 않습니다.
- CLI: `infer_cloud`, `infer_depth`, `infer_rgbd`. 설정은 `config/bridge.yaml`.
- `inference.launch.py`는 편의를 위해 SAM ROS bridge도 함께 시작하며 로봇 이동은 시작하지 않습니다.

[전체 패키지/API 안내](../docs/model-bridges.md)

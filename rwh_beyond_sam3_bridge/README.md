# rwh_beyond_sam3_bridge

독립 SAM ROS 인터페이스입니다. GPU 모델은 기존 `rwh-sam3-local` 컨테이너에 유지합니다.

```bash
ros2 launch rwh_beyond_sam3_bridge bridge.launch.py endpoint:=tcp://127.0.0.1:5567
```

- 노드: `sam3_bridge`, 실행 파일: `segmentation_action_server`.
- Action `/segment_image`: `rwh_beyond_msgs/action/SegmentImage`.
- Service `/sam3/health`: `std_srvs/srv/Trigger`.
- 입력: RGB와 text/box/selection point. 출력: depth-safe mask·score·instance_count.
- mock은 `allow_mock=false`가 기본이며 자동 fallback은 없습니다.
- `action_server.py`: ROS 서버. `segmentation_client.py`: SAM ZMQ 요청. `action_client.py`: ROS client.
- 설정은 `config/bridge.yaml`. MoveIt·coordinator·GraspGenX 없이 실행 가능합니다.

[전체 패키지/API 안내](../docs/model-bridges.md)

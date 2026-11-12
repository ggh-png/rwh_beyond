# 모델별 ROS 인터페이스

| ROS 노드 | Action / 서비스 | 역할 |
|---|---|---|
| `sam3_bridge` | `/segment_image`, `/sam3/health` | RGB·대상 조건 → 마스크 |
| `graspgenx_bridge` | `/infer_grasps`, `/graspgenx/health` | 객체 XYZ → 파지 후보 |
| `grasp_coordinator` | `/grasp/*`, `/grasp/cleanup` | 관측·계획·실행 상태 관리 |

두 health 서비스는 `std_srvs/srv/Trigger`입니다. 실제 모델 프로세스가 GPU에 준비되어 있어야 합니다.
ROS bridge가 실행 중이라는 사실만으로 GPU 모델 준비를 확인할 수는 없습니다.

## 기본 BT 경로

`SegmentObject`가 `/segment_image`를 호출하고, `BuildObjectCloud`가 로컬 CPU에서
Depth를 XYZ로 변환합니다. `InferGrasps`는 `/infer_grasps`의 POINT_CLOUD 모드로 요청합니다.
모델 통신은 각 bridge의 ZeroMQ 클라이언트가 담당합니다.

`bt_demo.launch.py`가 pipeline launch를 포함하므로 별도 bridge를 중복 실행하지 않습니다.
모든 BT 노드는 `rwh_beyond_bringup/config/bt_demo.yaml`을 공유합니다.

## 독립 추론

모델별 bridge만 실행할 수 있습니다. 이 명령은 로봇 동작을 보내지 않습니다.

```bash
ros2 launch rwh_beyond_sam3_bridge bridge.launch.py endpoint:=tcp://127.0.0.1:5567
ros2 launch rwh_beyond_graspgenx_bridge bridge.launch.py endpoint:=tcp://127.0.0.1:5566
```

`SegmentImage`는 RGB, target_text/box, 인스턴스 선택 조건을 받습니다.
`InferGrasps`는 다음 입력 모드를 제공합니다.

| 모드 | 입력 | 처리 |
|---|---|---|
| `POINT_CLOUD=0` | 객체 XYZ PointCloud2 | GraspGenX 호출 |
| `DEPTH_IMAGE=1` | Depth·CameraInfo·마스크 | 로컬 역투영 후 GraspGenX |
| `RGBD=2` | 정렬 RGB-D·CameraInfo·대상 조건 | SAM Action → 로컬 역투영 → GraspGenX |

RGBD 편의 모드는 독립 호출용입니다. 기본 BT는 이미 분리한 점군을 사용하므로 SAM을 중복 호출하지 않습니다.

[데이터·좌표계](project-data.md) · [런타임 endpoint](piper-runtime.md)

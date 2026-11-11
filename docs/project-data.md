# 모델과 어떤 데이터를 주고받나

기본 BT 경로는 **SAM에 RGB만 보내 분할하고, ROS에서 Depth를 XYZ로 바꾼 뒤,
GraspGenX에 객체 XYZ만 보내 파지 자세를 받는 구조**입니다.

## 1. 카메라 → ROS 관측

coordinator가 구독하는 기본 토픽입니다. 시뮬레이션에서는 MuJoCo가 제공합니다.

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | SAM 입력 RGB |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 픽셀별 실제 깊이 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 역투영용 내부 파라미터 K |
| `/feedback/joint_states` | `sensor_msgs/JointState` | 촬영 후 팔 이동 여부 확인 |
| `/tf`, `/tf_static` | TF | 촬영 시각의 카메라 → base 변환 |
| `/camera/depth/filtered_points` | `sensor_msgs/PointCloud2` | 충돌 지도 갱신 확인 |

RGB와 Depth는 같은 해상도·같은 optical frame으로 정합되어야 합니다. 단순히 컬러/Depth
토픽 이름만 바꾸면 서로 맞는 영상이 된다는 뜻은 아닙니다. coordinator는 같은 stamp의 RGB·Depth·Info를 모읍니다.

## 2. ROS → SAM → ROS

ZMQ 주소 `tcp://127.0.0.1:5567`. 아래는 배열을 생략한 요청 구조 예시입니다.

```python
{
    "action": "segment",
    "rgb": rgb_array,               # uint8 [H, W, 3], RGB 순서
    "text": "red box",
    "box": [],                      # 선택: pixel [xmin, ymin, xmax, ymax]
    "selection_point": [],          # 선택: pixel [u, v]
    "selection_policy": "unique"
}
```

| SAM 응답 필드 | 의미 |
|---|---|
| `mask` | `uint8 [H,W]`, 객체 1 / 배경 0 |
| `score` | 선택 객체의 분할 점수 0~1 |
| `instance_count` | 검출 인스턴스 수 |
| `backend`, `mock` | 실제 `sam3`, `False`인지 확인 |
| `elapsed` | 서버 분할 처리 시간(초) |

Depth·TF·관절값은 SAM에 보내지 않습니다. ROS client는 마스크를 3×3 침식 1회 처리해
경계의 배경 Depth 혼입을 줄입니다. [공식 SAM API를 호출하는 방법](sam3-perception.md)도 참고하세요.

## 3. 마스크 + Depth + K → 객체 점군

ROS CPU의 `depth_converter.py::depth_xyz()`가 마스크 안의 유효한 깊이 픽셀만 역투영합니다.
일반적인 pinhole 표현은 아래와 같습니다. 실제 코드는 K 역행렬을 사용합니다.

```text
Z = depth(u,v)
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

`16UC1` Depth의 mm는 m로 변환하고, `32FC1`은 m로 취급합니다. NaN·0 이하 깊이를 제외합니다.
결과는 **float32 [N,3], 미터, 카메라 optical frame**의 XYZ이며 색상·삼각형 mesh는 포함하지 않습니다.
ROS에서는 `PointCloud2`로 보관하고 `/grasp/object_cloud`에 발행합니다.

## 4. ROS → GraspGenX → ROS

coordinator → `/infer_grasps` ROS Action → inference bridge → ZMQ `tcp://127.0.0.1:5566` 순서입니다.
기본 BT가 만드는 Action goal의 입력 모드는 `POINT_CLOUD`입니다.

```python
{
    "action": "infer",
    "point_cloud": xyz,             # float32 [N,3], 미터
    "gripper_name": "piper_hand",
    "num_grasps": 100,
    "grasp_threshold": -1.0,
    "topk_num_grasps": 100
}
```

| GraspGenX 주요 응답 | 의미 |
|---|---|
| `grasps` | `[K,4,4]` 파지 위치·방향 동차 행렬 |
| `confidences` | `[K]` 파지 후보 점수 0~1 |
| `gripper_name` | 요청한 그리퍼와 일치하는지 확인 |

ZMQ 요청에는 ROS `frame_id`·stamp를 넣지 않습니다. Bridge가 원본 PointCloud2 header를
보관하고 반환 후보에 다시 붙입니다. 반환 자세는 **입력 점군 좌표계 기준의 모델 그리퍼 자세**입니다.
Bridge는 행렬·회전·점수 범위를 검증하고 점수 내림차순으로 정렬해 최종 `top_k`를 적용합니다.
ROS 결과는 `GraspCandidate[]`: `id`, `PoseStamped pose`, `float32 score`입니다.

통신 인코딩은 JSON 문자열이 아니라 **MessagePack + msgpack-numpy**입니다.
요청마다 ZMQ REQ socket을 만들고 응답을 기다립니다. 오류는 `error` 응답이나 timeout으로 처리합니다.
클라이언트 취소는 대기를 끝내고 socket을 닫지만 원격 GPU 계산 자체를 중단시키지는 않습니다.

## 5. 파지 행렬 → 로봇 TCP 목표

선택기가 촬영 당시 TF와 그리퍼 보정값을 적용합니다.

```text
base_T_tcp = base_T_camera × camera_T_model_grasp × model_T_tcp
```

- `base_T_camera`: 촬영 순간 TF. 이후 카메라가 움직였을 때의 TF로 바꾸지 않습니다.
- `camera_T_model_grasp`: GraspGenX의 후보 행렬.
- `model_T_tcp`: 모델 그리퍼 원점/축에서 실제 TCP로의 보정.

현재 보정은 `model_to_tcp_xyz=[0,0,0.13]`m, `model_to_tcp_rpy=[0,0,-π/2]`rad입니다.
이 값은 URDF의 flange→TCP 설정인 `tcp_offset`과 서로 다른 변환이므로 그대로 대입해서 바꾸면 안 됩니다.
선택기는 SAM 경로에서 관측한 물체 AABB·수평 지지면으로 충돌 객체를 만들고,
하향 접근 조건·IK·충돌·Cartesian 접근/리프트·사전 접근 경로를 검사합니다.
모델 점수가 가장 높은 후보가 실제 선택된 후보와 항상 같지는 않습니다.

## 6. BT 데이터와 RViz 데이터

| 데이터 | 처리에 사용하는 값 | RViz 표시 |
|---|---|---|
| 마스크 | Blackboard Image, 픽셀 0/1 | `/grasp/object_mask_preview`는 0/255, `/grasp/segmentation_overlay`는 RGB 합성 |
| 점군 | Blackboard PointCloud2, camera frame | `/grasp/object_cloud_base`는 촬영 TF로 고정한 base 점군 |
| 후보 | Blackboard GraspCandidate 배열 | `/grasp/grasp_candidates` MarkerArray |
| 선택 결과 | 서버의 경로 + Blackboard `plan_id`·`pregrasp` | `/grasp/selected_tcp`, `/grasp/selected_grasp` |

`observation_id`는 한 번의 촬영과 데이터를 묶습니다. 촬영 후 30초 초과 또는 팔 관절 0.02 rad 초과
변화 시 재관측을 요구합니다. `plan_id`는 서버에 저장된 선택 경로를 참조합니다.
마스크·점군·ID가 같은 관측인지 검사하므로 임의 데이터를 넣는 무상태 서비스는 아닙니다.
RViz 토픽은 디버깅 출력이며 다음 BT 단계가 토픽에서 결과를 다시 읽는 구조는 아닙니다.

## 7. 독립 추론 API와 기본 BT의 차이 {#standalone}

`rwh_beyond_msgs/action/InferGrasps.action`은 세 입력 모드를 지원합니다.

| 모드 | 입력 | Bridge 처리 |
|---|---|---|
| `POINT_CLOUD=0` | 분리된 객체 점군 | 바로 GraspGenX 호출. **기본 BT 경로** |
| `DEPTH_IMAGE=1` | Depth·CameraInfo·객체 마스크 | 로컬 역투영 후 GraspGenX |
| `RGBD=2` | RGB·Depth·CameraInfo·대상 조건 | SAM → 로컬 역투영 → GraspGenX |

RGBD 편의 API는 독립 클라이언트용이며 기본 BT가 SAM 호출을 중복 수행하는 것은 아닙니다.
`inference.launch.py`만 실행한 추론 서버는 로봇 이동을 보내지 않습니다.

다음: [처음 터미널부터 실행](grasp-pipeline-guide.md)

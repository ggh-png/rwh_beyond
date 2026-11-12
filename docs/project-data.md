# 데이터와 좌표계

## RGB-D와 물체 마스크

coordinator는 같은 timestamp의 RGB·정렬 Depth·CameraInfo를 수집하고 촬영 시각의 TF를 보관합니다.
SAM 3에는 RGB와 물체 설명을 전달합니다. 반환된 한 물체의 binary 마스크를 Depth에 적용합니다.

| 데이터 | 형식 |
|---|---|
| RGB | `sensor_msgs/Image`, rgb8/bgr8 |
| Depth | `16UC1`: mm, `32FC1`: m |
| CameraInfo | 정렬 Depth에 대응하는 pinhole intrinsics |
| 물체 마스크 | mono8 binary, 원본 영상과 동일한 해상도·header |
| 객체 점군 | `sensor_msgs/PointCloud2`, XYZ, m, 카메라 optical frame |

마스크 밖과 유효하지 않은 Depth를 제외하고 로컬 CPU에서 역투영합니다.
마스크 경계를 침식해 배경 깊이가 물체 점군에 섞이는 것을 줄입니다.
GraspGenX는 RGB나 텍스트가 아닌 `float32 [N,3]` XYZ 점군을 받습니다.

## 파지 후보와 TCP

GraspGenX 응답은 파지 변환 행렬·점수이며 bridge가 `GraspCandidate[]`로 변환합니다.
각 후보의 frame과 timestamp는 입력 점군을 유지합니다.

```text
base_T_tcp = base_T_camera × camera_T_model_grasp × model_T_tcp
```

- `base_T_camera`: 촬영 순간 TF.
- `camera_T_model_grasp`: 모델이 반환한 파지 자세.
- `model_T_tcp`: 모델 그리퍼 좌표에서 로봇 TCP로의 보정.

공통 YAML의 `model_to_tcp_xyz=[0,0,0.13]` m,
`model_to_tcp_rpy=[0,0,-pi/2]` rad를 사용합니다.
이 값은 flange에서 TCP까지의 URDF `tcp_offset`과 다른 변환입니다.

선택기는 후보 점수뿐 아니라 하향 접근 조건·TCP 보정·IK·충돌·Cartesian 접근/리프트·
사전 접근 경로를 검사합니다. 모든 후보가 탈락하면 실행하지 않습니다.

## 관측 기반 충돌 장면

SAM 경로는 객체 점군과 주변 Depth에서 물체 AABB와 수평 지지면을 추정합니다.
명목 박스와 테이블을 Planning Scene에 추가하고 대상 영역을 계획용 점군에서 제외합니다.
현재는 수평 지지면을 가진 tabletop 장면을 전제로 합니다.

닫기 후에는 같은 물체를 TCP에 부착된 충돌 객체로 취급합니다.
이는 물체가 실제로 잡혔다는 측정 결과가 아닙니다.

## 관측·계획 ID

`observation_id`는 한 번의 촬영과 마스크·점군·후보를 연결합니다.
촬영 후 30초 초과 또는 관절 0.02 rad 초과 변화는 재관측을 요구합니다.
`plan_id`는 coordinator에 보관한 선택 경로와 단계 진행 상태를 참조합니다.

RViz의 base 점군·마커는 디버그 출력입니다. 다음 BT 단계는 토픽에서 결과를 다시 읽지 않고
Action 결과와 Blackboard 데이터를 사용합니다.

[단계별 포트](bt-grasp-stages.md) · [카메라 TF](d405-sensors.md)

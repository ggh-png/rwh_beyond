# 처음 터미널부터 실행하기

이 페이지는 **현재 PC에 준비된 컨테이너와 빌드**로 MuJoCo 집기 실험을 실행하는 절차입니다.
전체 구조는 [한눈에 보기](project-overview.md), 이미지 생성 원리는 [컨테이너 구축](project-containers.md)을 참고하세요.

> 현재 workspace 기준 runner 이름은 **`bt_demo_sim`**입니다. 이전 대화·기록의
> `activate_bt_demo` 대신 **`activate_bt_demo_sim`**을 사용합니다.
> 실제 MuJoCo runner는 **`piper_sim`**입니다. `piper_moveit_sim`은 현재 실물 runner의 기존 이름이므로
> 시뮬레이터 활성화 명령으로 사용하지 마세요. 실물 MoveIt 시험은 [별도 절차](piper-hardware-moveit.md)를 사용합니다.
> 아래에서는 `node_name:=ros_runner`로 통일하므로 서비스 prefix는 `/ros_runner`입니다.

## 0. 터미널 역할

| 터미널 | 위치 | 실행할 것 |
|---|---|---|
| A | 호스트 → SAM 컨테이너 | SAM health 확인, 필요할 때 모델 서버 유지 |
| B | 호스트 → GraspGenX 컨테이너 | Grasp health 확인, 필요할 때 모델 서버 유지 |
| C | 호스트 → ROS 컨테이너 | runner 실행·로그 확인 |
| D | 호스트 → ROS 컨테이너 | 활성화, BT goal, 상태 확인·종료 |

이미 runner나 수동 simulation/Bridge가 실행 중이라면 같은 구성을 중복 실행하지 않습니다.
아래 명령은 호스트의 `docker` 호환 명령을 `sudo` 없이 사용합니다.

## 1. 호스트에서 컨테이너 켜기

```bash
# ROS 컨테이너가 두 모델 컨테이너의 네트워크 기준이므로 먼저 시작
docker start moma_mj_lab_1.1.2_hostcan
docker start rwh-sam3-local rwh-graspgenx-local
docker ps
```

이 명령은 기존 컨테이너를 시작합니다. 이미지 build·모델 다운로드는 매 실험마다 필요하지 않습니다.

## 2. 터미널 A: SAM 모델 준비

호스트에서 접속합니다.

```bash
docker exec -it rwh-sam3-local bash
```

**SAM 컨테이너 안에서** 상태를 확인합니다.

```bash
cd /opt/segmentation
python -c 'from wire import request; print(request("tcp://127.0.0.1:5567", {"action":"health"}, 3))'
```

`status: ok`, `backend: sam3`, `mock: False`이면 다음으로 넘어갑니다.
응답이 없고 서버가 꺼져 있을 때만 시작합니다.

```bash
python -u /opt/segmentation/server.py
```

서버를 시작한 터미널은 유지합니다. `Ctrl+C`는 이 모델 프로세스를 종료합니다.

## 3. 터미널 B: GraspGenX 모델 준비

호스트에서 접속합니다.

```bash
docker exec -it rwh-graspgenx-local bash
```

**GraspGenX 컨테이너 안에서** 확인합니다.

```bash
python /opt/deploy/scripts/health_check.py --endpoint tcp://127.0.0.1:5566
```

`GraspGenX healthy`이면 다음으로 넘어갑니다. 서버가 꺼져 있을 때만 시작하고 터미널을 유지합니다.

```bash
bash /opt/deploy/scripts/start_server.sh
```

두 health 검사가 모두 성공해야 합니다. 컨테이너 `Up`과 모델 `healthy`는 다른 상태입니다.

## 4. 터미널 C: ROS runner 실행

호스트에서 접속합니다.

```bash
docker exec -it moma_mj_lab_1.1.2_hostcan bash
```

**ROS 컨테이너 안에서** 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RWH_PIPER_SIM_GUI=true
export RWH_BT_RVIZ=true
export RWH_SEGMENTATION_BACKEND=sam3
export RWH_GRASP_TARGET_TEXT="red box"

ros2 launch ros_runner runner.launch.py   node_name:=ros_runner   config_yaml:=/root/ros2_ws/src/ros_runner/config/runners.yaml
```

이 터미널을 유지합니다. 환경변수는 자식 launch에 상속되므로 runner **시작 전** 설정합니다.
runner 설정을 바꿨다면 기존 runner를 종료한 뒤 다시 켜야 합니다.

## 5. 터미널 D: 시뮬레이터·BT·통합 RViz 활성화

호스트에서 같은 ROS 컨테이너에 추가 접속합니다.

```bash
docker exec -it moma_mj_lab_1.1.2_hostcan bash
```

이 터미널도 ROS 환경과 도메인을 맞춥니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 service call /ros_runner/activate_bt_demo_sim std_srvs/srv/Empty "{}"
ros2 lifecycle get /bt_navigator
```

navigator가 `active [3]`이 될 때까지 기다립니다. 아직 시작 중이면 lifecycle 명령을 다시 확인합니다.

```text
ros_runner: bt_demo_sim
    ├─ requires: piper_sim
    │     └─ MuJoCo + MoveIt (rviz:=false)
    └─ bt_demo.launch.py
          ├─ grasp_pipeline.launch.py → SAM bridge + GraspGenX bridge + coordinator
          ├─ BT navigator + lifecycle manager
          └─ MoveIt/SAM/점군/파지 통합 RViz
```

RViz는 BT launch에서 기본 실행합니다. `RWH_BT_RVIZ=false`일 때만 끕니다.
이전 `RWH_PIPER_SIM_RVIZ` 대신 현재 runner에서는 `RWH_BT_RVIZ`를 사용합니다.
GPU 모델은 runner가 자동 시작하지 않습니다.

## 6. 먼저 영상·점군 확인

터미널 D에서 실행합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo   "{behavior_tree: /root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/perception_preview.xml}"   --feedback
```

홈·그리퍼 열기·관측 자세까지 **팔이 움직입니다**. SAM 분할과 객체 점군까지만 생성하고 집지는 않습니다.
RViz에서 마스크·RGB overlay·객체 점군을 확인합니다. 결과는 연속 동영상 분할이 아니라
**BT가 관측한 순간의 스냅샷**입니다. coordinator가 살아 있으면 늦게 연 RViz도 마지막 결과를 받습니다.

## 7. 파지 후보·선택 결과까지만 확인

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo   "{behavior_tree: /root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/grasp_debug.xml}"   --feedback
```

새 관측 → 점군 → GraspGenX → 실행 가능 후보 선택까지 수행합니다.
최종 접근·닫기·리프트는 하지 않습니다. 후보/점수·선택 TCP·pregrasp를 RViz에서 볼 수 있습니다.
후보 표시는 기본 최대 10개이며 실제 추론·선택 후보 개수와 별도입니다.

## 8. 전체 집기

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo "{}" --feedback
```

빈 goal은 기본 `demo.xml`을 사용합니다. 접근·닫기·접촉 확인·리프트·홈 복귀까지 실행합니다.
성공 후에도 그리퍼는 닫혀 있습니다. 반복 실험은 아래 초기화 절차를 사용하세요.

## 9. 다른 물체 지정 {#target}

대상은 `action` 문자열이 아니라 `target_text`로 바꿉니다.
BT 실행이 끝난 상태에서, 다음 관측 **전에** 설정합니다.

```bash
ros2 param set /grasp_coordinator target_text "blue cup"
ros2 param get /grasp_coordinator target_text
```

또는 터미널 C의 runner 시작 전에 `RWH_GRASP_TARGET_TEXT="blue cup"`을 지정합니다.
이 환경변수는 재시작 시 초기값이며 실행 중의 파라미터 변경은 재시작 후 유지되지 않습니다.
`/run_demo` goal에는 현재 대상 텍스트 필드가 없고 `behavior_tree`만 있습니다.

복수 객체 구분이 필요하면 원본 영상의 pixel 박스나 선택점을 지정할 수 있습니다. 아래 숫자는 예시입니다.

```bash
ros2 param set /grasp_coordinator target_box "[100.0, 60.0, 220.0, 180.0]"
ros2 param set /grasp_coordinator selection_point "[160, 120]"
# 다시 텍스트만 사용할 때는 두 제한을 모두 해제
ros2 param set /grasp_coordinator target_box "[0.0, 0.0, 0.0, 0.0]"
ros2 param set /grasp_coordinator selection_point "[-1, -1]"
```

기본 `unique`는 모호한 다중 검출을 거부합니다. 점은 SAM의 직접 point prompt가 아니라
검출된 mask 중 해당 픽셀을 포함하는 객체를 고릅니다.

**SAM 경로는 관측 점군에서 물체 크기·지지면을 추정합니다.** 고정 `object_dimensions`는
`color_demo`에서 사용합니다. 현재 SAM 실험에서 다른 물체를 고르기 위해 이 값을 매번 바꾸는 구조는 아닙니다.
다만 시뮬레이션 장면에 그 물체가 존재해야 하고, 현재 하향 파지 선택 조건·그리퍼 크기·접촉 fixture 제약도
만족해야 합니다. 대상 텍스트 변경은 임의 물체의 집기 성공을 보장하지 않습니다.

## 10. 초기화와 종료

BT goal이 끝난 뒤 시뮬레이션을 초기화하고 다시 활성화합니다.

```bash
ros2 service call /ros_runner/deactivate_piper_sim std_srvs/srv/Empty "{}"
ros2 service call /ros_runner/activate_bt_demo_sim std_srvs/srv/Empty "{}"
ros2 lifecycle get /bt_navigator
```

실험을 종료하려면 `deactivate_piper_sim` 호출 후 runner 터미널 C에서 `Ctrl+C`를 누릅니다.
모델 서버는 별도이므로 유지하거나, 직접 시작했던 A/B에서 각각 `Ctrl+C`로 종료합니다.
컨테이너도 끄려면 호스트에서 다음 순서로 실행합니다.

```bash
docker stop rwh-sam3-local rwh-graspgenx-local
docker stop moma_mj_lab_1.1.2_hostcan
```

## 문제를 어느 단계에서 확인하나

| 증상 | 먼저 확인 |
|---|---|
| 모델 연결 실패 | A/B의 health, 모델 프로세스, [공유 네트워크](project-containers.md) |
| 활성화 서비스 없음 | runner 이름·실행 여부, `ros2 service list`, 도메인 42 일치 |
| `active`가 안 됨 | C 로그, simulation clock·MoveIt·Bridge 준비 상태 |
| RViz 안 뜸 | `RWH_BT_RVIZ`, GUI 환경, 새 runner 설정 재시작 |
| 영상/점군 없음 | 관측 BT 실행 여부, target 검출, [표시 토픽/QoS](grasp-visualization.md) |
| `START_STATE_INVALID` | 해당 이동의 시작 상태·Planning Scene·충돌 상태. 다음 단계는 실행되지 않음 |
| `No feasible model grasp` | 선택기의 rejection counts: IK·하향 조건·충돌·접근/리프트 경로 |
| `acquire_object_rgbd` timeout | 먼저 확인: coordinator의 `use_sim_time`, 카메라 3종 수신·동일 stamp. 아래 시간 설정 절 참고 |
| 오래된 관측/팔 이동 | 새 BT goal로 재관측. 오래된 ID·선택 경로 재사용 금지 |
| 이미 물체를 잡고 있음 | 현재 트리에는 내려놓기가 없으므로 시뮬레이션 초기화 |

이 문서 개편에서 새 물리 집기 시험은 수행하지 않았습니다. 과거 시험의 성공·실패는
[검증 범위](project-overview.md#validation)에서 구분해 확인하세요.


### RGB-D 수신 timeout과 시간 설정

`acquire_object_rgbd`는 SAM 요청 전 단계다. RGB·Depth·CameraInfo가 수신되어도
coordinator가 시스템 시간을 사용하고 카메라가 시뮬레이션 시간을 쓰면 새 관측 조건을 만족할 수 없다.

```bash
ros2 param get /grasp_coordinator use_sim_time
ros2 param get /graspgenx_bridge use_sim_time
```

MuJoCo에서는 `True`, 실물에서는 `False`여야 하며 BT launch의 `mode`가 설정한다.
BT launch는 `rwh_beyond_bringup/config/bt_demo.yaml`을 모든 BT 관련 노드에 공통으로 전달한다.
`params_file:=...`로 전체 YAML을 교체한다.
패키지·Action·설정 파일 대응은 [ROS bridge 분리](model-bridges.md)를 참고한다.
수정 후에는 관련 패키지를 빌드하고 기존 runner를 종료·재시작한다. 이 문제 때문에 SAM 컨테이너를
재시작할 필요는 없다.

## 원격 RTX 5090 추론 품질 설정 (2026-11-11)

`rwh_beyond_grasp_pipeline/config/grasp_pipeline.yaml`의 기본 설정을 다음과 같이
조정했습니다. server 컨테이너 재시작은 필요하지 않으며, 요청을 보내는 ROS pipeline을
재시작하면 반영됩니다.

| 파라미터 | 기존 | 현재 | 의미 |
|---|---:|---:|---|
| `num_grasps` | 100 | 1000 | 모델에서 생성할 파지 표본 수 |
| `top_k` | 100 | 300 | 점수순으로 ROS에 전달할 최대 후보 수 |
| `max_candidates` | 100 | 300 | MoveIt 타당성 검사에 사용할 최대 후보 수 |

모델 후보는 기존과 같이 점수순으로 정렬하고, 접근 방향·대상 위치·테이블 여유·IK·접근 및
리프트 경로·충돌 계획을 통과한 첫 후보를 선택합니다. 후보를 늘려도 검사 기준이나
속도, TCP, 충돌 여유를 완화하지 않습니다. RViz 후보 표시 개수는 10개로 유지합니다.

저장된 단일 시뮬 RGB-D의 같은 분할 점군으로 각 설정을 5회 비교했습니다.
생성 100 → 1000개에서 상위 20개 평균 모델 점수는 약 0.737 → 0.817,
위에서 접근하는 후보의 최고 점수 평균은 약 0.735 → 0.812였습니다.
GraspGenX 왕복 평균 시간은 약 0.164 → 0.255초였습니다.
이 점수는 학습 모델의 confidence이며 실제 파지 성공률이나 분할 정확도를 뜻하지 않습니다.
[비교 원자료](validation/remote-grasp-tuning-2026-11-11.json)를 참고하세요.

SAM confidence threshold 0.5와 학습 모델의 입력 처리, 모호한 다중 객체를 거부하는
`unique` 정책은 유지합니다. 정답 마스크나 여러 실물 물체의 성공률 데이터 없이
임계값을 높여 정확도가 개선됐다고 판단하지 않습니다.

기존 설정으로 돌아가려면 위 세 값을 각각 100으로 바꾸고 pipeline을 재시작합니다.
현재 GraspGenX bridge의 `max_grasps`는 1000이므로 `num_grasps`를 더 높이려면
bridge 제한과 서버 GPU 사용량을 먼저 검증해야 합니다.

설정 적용 후 초기 시뮬레이션을 매번 다시 시작해 BT를 3회 시험했습니다.
2회는 집기·물리 리프트·홈 복귀까지 SUCCESS였고, 1회는 추론 이전 MoveObservation에서
joint3가 상한을 약 `0.000032 rad` 넘어 START_STATE_INVALID로 중단됐습니다.
이를 파지 추론 성공률로 해석하지 않으며, 관절 제한 변경이나 feedback clamp는 적용하지
않았습니다. 마지막 시험에서 ROS 후보 300개 전달도 확인했습니다.
[시뮬레이션 시험 결과](validation/remote-grasp-tuning-simulation-2026-11-11.json)를 참고하세요.

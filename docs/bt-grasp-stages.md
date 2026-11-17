# BT 플러그인별 기능과 포트

16개 노드의 **역할 → 처리 순서 → 입출력 → 호출·오류**를 찾아보는 참조 문서입니다.
전체 실행 순서는 [BT 실행 흐름](bt-flows.md), 프로세스 관계는 [BT와 실행 구조](bt-architecture.md)를 참고하세요.

<div class="bt-plugin-reference" markdown="1">

## 노드 바로 찾기

<div class="bt-plugin-index" markdown="1">

- **준비·관측**
  [CheckGraspReady](#checkgraspready) · [ClearOctomap](#clearoctomap) · [MoveHome](#movehome) · [OpenGripper](#opengripper) · [MoveObservation](#moveobservation)
- **촬영·인식**
  [CaptureRgbd](#capturergbd) · [SegmentObject](#segmentobject) · [BuildObjectCloud](#buildobjectcloud) · [InferGrasps](#infergrasps)
- **파지 선택·실행**
  [SelectFeasibleGrasp](#selectfeasiblegrasp) · [MoveToPregrasp](#movetopregrasp) · [ApproachGrasp](#approachgrasp) · [AttachObject](#attachobject) · [LiftObject](#liftobject)
- **배치·해제**
  [MovePlace](#moveplace) · [ReleaseObject](#releaseobject)

</div>

각 카드의 **처리 흐름**은 읽는 순서, **입출력**은 XML 포트,
**호출·상세 규칙**은 ROS 인터페이스·제한·실패 조건입니다.
`{이름}`은 Blackboard 키이며, `SUCCESS`는 해당 노드의 완료 조건을 충족했다는 뜻입니다.


## 준비·지도·관측 이동

<div class="bt-plugin" markdown="1">

### CheckGraspReady

<span class="bt-plugin-kind">준비 검사</span>

집기를 시작할 수 있는지 로봇·장면·모델의 준비 상태를 확인합니다.

!!! note "알아둘 점"

    `release_previous=true`는 실제 개방 명령입니다. 이전 물체를 현재 위치에서 놓습니다.

=== "처리 흐름"

    1. 최신 관절 피드백을 최대 5초 기다리고 로봇 모델·관절 한계·실물 전송 경로를 확인합니다.

    2. Planning Scene의 부착 상태를 확인합니다. `release_previous=true`이면 이전 대상을 현재 위치에서 개방·해제합니다.

    3. SAM·GraspGenX health 응답과 MoveIt·그리퍼 인터페이스 준비를 확인합니다.

    **완료 조건**

    모션과 인식 요청을 보낼 준비가 됐습니다. 촬영·목표 물체 확인은 이후 노드가 수행합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `release_previous` | false | true이면 이전 대상을 현재 위치에서 개방·해제 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/get_planning_scene`, `/sam3/health`, `/graspgenx/health`.
      실물에서는 `/grasp/hardware_transport_ready`의 `physical_feedback_transport` 응답도 확인합니다.

    - **부착 처리:** `false`이면 기존 대상이 부착된 상태에서 새 pick을 거부합니다.
      `true`이면 현재 위치에서 그리퍼 개방·부착 해제·정리 후 진행합니다.
      알 수 없는 부착 물체는 거부합니다.

    - 촬영은 수행하지 않습니다. 실제 RGB-D 획득은 뒤의 `CaptureRgbd`가 담당합니다.

    **실패를 볼 때.** 오래된 피드백, 실물 transport 응답 실패, 알 수 없는 부착 물체,
    모델 health 실패, MoveIt·그리퍼 준비 시간 초과를 구분해 확인합니다.
    `release_previous=true`는 단순 상태 초기화가 아니라 실제 개방 명령을 포함합니다.

</div>

<div class="bt-plugin" markdown="1">

### ClearOctomap

<span class="bt-plugin-kind">지도 갱신</span>

기존 Octomap을 지우고 새 깊이 데이터로 지도가 다시 만들어질 때까지 기다립니다.

!!! note "알아둘 점"

    현재 `octomap_enabled=false`이면 즉시 성공으로 건너뜁니다. clear 응답만 받은 상태는 지도 준비 완료가 아닙니다.

=== "처리 흐름"

    1. **서비스 대기 · SERVICES** — clear·Planning Scene 서비스가 준비될 때까지 기다립니다.

    2. **초기화 요청 · CLEAR** — clear를 한 번 전송하고 응답을 기다립니다.

    3. **새 점군 수집 · CLOUDS** — clear 응답 이후의 서로 다른 비어 있지 않은 점군을 `fresh_clouds`개 수집합니다.

    4. **지도 확인 · SCENE** — Octomap을 조회합니다. 지도가 비어 있으면 새 점군을 더 기다린 뒤 다시 조회합니다.

    **완료 조건**

    새 점군과 재생성된 비어 있지 않은 Octomap을 모두 확인했습니다.

    ```mermaid
    flowchart LR
      A["서비스 대기"] --> B["clear 1회"] --> C["새 점군 수집"] --> D{"지도 재생성?"}
      D -->|예| E["SUCCESS"]
      D -->|아직 비어 있음| C
    ```

    **XML 예시**

    ```xml
    <ClearOctomap once="false" timeout="30.0" fresh_clouds="2"/>
    ```

=== "입출력"

    | 포트 | 기본값 | 의미 |
    |---|---|---|
    | `once` | `true` | 이 노드 인스턴스의 첫 성공 이후 생략 |
    | `timeout` | `30.0`초 | 서비스 대기부터 지도 확인까지 전체 제한 시간; 0 초과 |
    | `fresh_clouds` | `2` | clear 이후 필요한 서로 다른 새 점군 수; 1 이상 |
    | `service_name` | `/clear_octomap` | 지도를 지우는 서비스 |
    | `cloud_topic` | `/camera/depth/filtered_points` | 로봇이 필터링된 깊이 XYZ 구독 토픽 |
    | `scene_service` | `/get_planning_scene` | 재생성된 Octomap을 조회하는 서비스 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    | 종류 | 대상 | 역할 |
    |---|---|---|
    | 서비스 · `std_srvs/srv/Empty` | `/clear_octomap` | 기존 지도 초기화 |
    | 구독 · `sensor_msgs/msg/PointCloud2` | `/camera/depth/filtered_points` | clear 이후 새 점군 확인 |
    | 서비스 · `moveit_msgs/srv/GetPlanningScene` | `/get_planning_scene` | Octomap 재생성 확인 |

    이름은 기본값이며 포트에서 바꿀 수 있습니다.

    **실패 원인**

    | 오류 | 확인할 것 |
    |---|---|
    | `INVALID_INPUT` | 서비스·토픽 이름, 양수 timeout, 1 이상의 fresh_clouds |
    | `OCTOMAP_TIMEOUT` | 로그의 대기 단계: 서비스 / clear 응답 / 새 점군 / 지도 조회 |
    | `OCTOMAP_SERVICE_ERROR` | 서비스 호출·응답 처리 예외 |

    **설정·취소 규칙**

    - 기본 집기 XML은 `once=false`로 두 번 호출합니다.
    - `once`의 성공 기록은 다른 노드 인스턴스와 공유하지 않습니다.
    - clear는 활성화당 한 번 전송합니다. 불명확한 timeout에 자동 재전송하지 않습니다.
    - halt는 이미 전송한 Empty 서비스를 취소하지 않으며 지도 준비 완료로 표시하지 않습니다.

</div>

<div class="bt-plugin" markdown="1">

### MoveHome

<span class="bt-plugin-kind">팔 이동</span>

현재 관절 상태에서 지정된 home 관절 자세까지 계획하고 이동합니다.

=== "처리 흐름"

    1. 실물 전송과 Planning Scene 부착 상태를 확인합니다.

    2. 현재 관절값이 home 허용 오차 안이면 이동을 생략합니다.

    3. arm 6축을 home 값으로 바꾼 목표 상태로 충돌 검사 관절 경로를 계획·실행합니다.

    4. 실제 관절 피드백으로 home 도착을 확인합니다.

    **완료 조건**

    home 허용 오차 안에 도착했습니다. 들고 있던 대상은 계속 유지합니다.

    **XML 예시**

    ```xml
    <MoveHome home_joint_positions="0;0;0;0;0;0"/>
    ```

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `home_joint_positions` | YAML | joint1~6 순서의 유한한 rad 6개 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** Planning Scene 확인, `/plan_kinematic_path` (`GetMotionPlan`),
      `/execute_trajectory` (`ExecuteTrajectory`).

    - XML 생략 시 `bt_motion.home_joint_positions`를 사용합니다. 들고 있는 대상의 충돌 형상은 유지합니다.

    **완료와 주의점.** 성공은 home 허용 오차 안에 도착했다는 뜻입니다. 이 노드는 그리퍼를 열지 않으므로
    집기 후 복귀에서도 물체를 유지합니다. 계획 실패, 실행 거부, 피드백 도착 시간 초과는 모두 실패입니다.

</div>

<div class="bt-plugin" markdown="1">

### OpenGripper

<span class="bt-plugin-kind">그리퍼</span>

그리퍼를 지정 간격으로 열고 실제 개방 피드백을 확인합니다.

!!! note "알아둘 점"

    부착된 대상을 놓으려면 배치 후 `ReleaseObject`를 사용합니다.

=== "처리 흐름"

    1. 대상이 부착돼 있으면 개방을 거부합니다.

    2. `gripper` 관절 하나의 궤적을 지정 폭·시간으로 보냅니다.

    3. 액션 성공 후 최대 10초 동안 실제 간격과 목표의 차이가 5mm 미만인지 확인합니다.

    **완료 조건**

    그리퍼 개방과 피드백 확인이 끝났습니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `opening` | YAML; 현재 0.10m | 전체 간격 0~0.10m |
    | 입력 | `duration` | 3초 | 0.001~30초 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/gripper_controller/follow_joint_trajectory` (`FollowJointTrajectory`).

    - 생략 시 `bt_motion.gripper_open_width` (현재 0.10m), 3초입니다.
      부착된 대상이 있으면 자동 개방을 거부합니다. 배치 후 개방은 `ReleaseObject`를 사용합니다.

    **완료와 주의점.** 성공하면 다음 관측·집기 준비로 진행할 수 있습니다. 액션 성공만으로 개방 완료를
    판정하지 않습니다. 부착 상태에서 실패하는 것은 잘못된 호출 순서를 막는 검사이며,
    그 상태를 해제하려면 배치 후 `ReleaseObject` 흐름을 사용합니다.

</div>

<div class="bt-plugin" markdown="1">

### MoveObservation

<span class="bt-plugin-kind">팔 이동</span>

home 기준 TCP에 위치·기울기 오프셋을 적용해 관측 자세로 이동합니다.

!!! note "알아둘 점"

    현재 자세에서 이동을 계획합니다. 목표 오프셋은 매번 home 기준이며 누적되지 않습니다.

=== "처리 흐름"

    1. home 관절 상태의 FK로 기준 TCP를 구합니다.

    2. 위치는 base 축 `(x,y,z)`를 더하고, 방향은 home 로컬 RPY를 합성합니다. `tilt_deg`는 Y 회전만 바꿉니다.

    3. 현재 상태에서 목표까지 완전한 Cartesian 경로를 계획·실행합니다.

    4. 센서 안정화를 위해 `settle_time`을 기다립니다.

    **완료 조건**

    관측 자세 이동과 안정화 대기가 끝났습니다. 촬영은 `CaptureRgbd`가 수행합니다.

    **XML 예시**

    ```xml
    <MoveObservation x="0.0" y="0.0" z="0.10" tilt_deg="40.0" settle_time="0.5"/>
    ```

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `home_joint_positions` | YAML | home 기준 arm 6축 관절값(rad) |
    | 입력 | `x`, `y` | 각 0m | base 축 위치 오프셋; 각각 −0.30~0.30m |
    | 입력 | `z` | YAML; 현재 0.20m | base Z 오프셋; −0.30~0.30m |
    | 입력 | `tilt_deg` | YAML; 현재 30° | home 로컬 Y 기울기; −90~90° |
    | 입력 | `settle_time` | YAML; 현재 0.5초 | 이동 후 안정화 대기; 0~30초 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/compute_fk` (`GetPositionFK`), `/compute_cartesian_path` (`GetCartesianPath`),
      `/execute_trajectory`.

    - X·Y·Z 허용 범위는 각각 -0.30~0.30m, `tilt_deg`는 -90~90°이며 Y 기울임만 덮어씁니다.
      `settle_time`은 0~30초입니다. X·Y 기본값은 0, 나머지는 YAML에서 읽습니다.

    - home 도착은 전제조건이 아닙니다. 부착 상태에서는 복귀용으로도 사용합니다.
      호출할 때마다 목표를 home 기준으로 계산하며 현재 위치에 오프셋을 누적하지 않습니다.

    **실행과 완료.** 현재 관절 상태에서 이 목표로 완전한 Cartesian 경로를 계획·실행하고,
    `settle_time`을 기다린 뒤 성공합니다. 같은 노드를 두 번 실행해도 오프셋을 두 번 더하지 않습니다.
    Cartesian 경로가 불완전하면 실패하며, 이 노드 자체는 촬영하지 않습니다.

</div>

## 촬영·분할·점군·모델

<div class="bt-plugin" markdown="1">

### CaptureRgbd

<span class="bt-plugin-kind">센서 수집</span>

새 RGB·Depth·CameraInfo와 최신 관절값을 하나의 관측으로 묶습니다.

=== "처리 흐름"

    1. 수집 버퍼를 초기화하고 노드 시작 시각보다 새로운 프레임을 기다립니다.

    2. 같은 timestamp의 RGB·Depth·CameraInfo를 묶고 해상도·optical frame을 검사합니다.

    3. 1초 이내 수신한 유한한 arm 6축 관절값을 함께 저장합니다.

    4. 불변 `capture`와 프레임 timestamp 기반 `observation_id`를 Blackboard에 출력합니다.

    **완료 조건**

    SAM·점군 생성·파지 선택이 공유할 동일 시점의 관측을 얻었습니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `timeout` | 10초 | 새 관측 대기; 0 초과~30초 |
    | 출력 | `capture` | Observation | 불변 RGB·Depth·CameraInfo·관절값·ID |
    | 출력 | `observation_id` | 문자열 | 프레임 timestamp 기반 관측 ID |

=== "호출·상세 규칙"

    - **구독:** `camera/color/image_raw`, `camera/depth/image_rect_raw`, `camera/depth/camera_info`,
      `feedback/joint_states`. 영상 토픽은 `bt_perception.rgb_topic/depth_topic/info_topic`으로 변경할 수 있습니다.

    - 모델·MoveIt을 호출하지 않습니다. 취소 시 미완성 촬영 묶음을 비웁니다.

    **완료와 실패.** 성공한 관측은 뒤의 SAM·역투영·후보 선택이 공유하는 기준입니다.
    동기화되지 않은 영상이나 오래된 관절값은 기다리다가 timeout이 되고,
    등록된 영상의 크기·frame 불일치는 실패합니다. 카메라 연결만으로 성공하지 않습니다.

</div>

<div class="bt-plugin" markdown="1">

### SegmentObject

<span class="bt-plugin-kind">모델 액션</span>

촬영 RGB와 대상 조건을 SAM에 보내 물체 마스크를 받습니다.

=== "처리 흐름"

    1. 요청 → XML → launch override → YAML 순으로 대상 텍스트를 선택합니다.

    2. 관측 RGB와 box·selection point·selection policy를 액션 goal에 채웁니다.

    3. SAM 결과의 성공 여부와 mock 허용 조건을 검사합니다.

    4. 깊이 픽셀 선택에 쓸 이진 마스크를 `mask`에 출력합니다.

    **완료 조건**

    원본 영상에 대응하는 물체 마스크를 받았습니다. 깊이와의 정합성은 다음 노드에서도 검사합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `capture` | 필수 | CaptureRgbd의 Observation |
    | 입력 | `target_text` | 생략 가능 | 요청 텍스트가 우선; 없으면 XML·설정 사용 |
    | 입력 | `server_name` | segment_image | Nav2 액션 서버 이름 |
    | 입력 | `server_timeout` | 기본 XML 1000ms | goal 요청 응답 대기; 추론 전체 제한과 별개 |
    | 출력 | `mask` | Image | 깊이 픽셀 선택용 이진 마스크 |

=== "호출·상세 규칙"

    - **직접 호출:** `/segment_image` (`rwh_beyond_msgs/action/SegmentImage`).

    - 대상 우선순위는 `/run_demo.target_text` → XML `target_text` → launch 대상 override → YAML 기본값입니다.
      박스·선택점·선택 정책은 `bt_perception`에서 읽습니다. 현재 정책은 `highest_score`입니다.

    - 표시용 잘린 이미지가 아니라 깊이 픽셀 선택에 사용할 이진 마스크를 출력합니다.
      BT의 결과 대기는 70초이며 취소를 브릿지에 전달합니다.

    **완료와 실패.** 액션 goal 거부·서버 중단·70초 결과 시간 초과·mock 거부는 실패입니다.
    마스크와 깊이의 정합성은 다음 `BuildObjectCloud`에서도 검사합니다.
    대상을 바꿔 실행하려면 `/run_demo.target_text`를 지정할 수 있으며, 요청 값은 XML 값보다 우선합니다.

</div>

<div class="bt-plugin" markdown="1">

### BuildObjectCloud

<span class="bt-plugin-kind">내부 계산</span>

물체 마스크와 깊이를 역투영해 카메라 좌표계 XYZ 점군을 만듭니다.

=== "처리 흐름"

    1. 마스크·깊이·보정의 frame·stamp·크기와 저장 형식을 검사합니다.

    2. 마스크가 선택한 유효 깊이 픽셀을 역투영합니다. `16UC1`은 mm, `32FC1`은 m입니다.

    3. 100개 이상 유효 점을 요구하고 최대 200000개로 줄입니다.

    4. `cloud` 출력과 `/grasp/object_cloud` 디버그 발행을 마칩니다.

    **완료 조건**

    카메라 optical frame의 XYZ float32 점군을 얻었습니다. base 변환은 후보 선택 때 수행합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `capture` | 필수 | CaptureRgbd의 Observation |
    | 입력 | `mask` | 필수 | 동일 관측의 mono8 또는 8UC1 마스크 |
    | 출력 | `cloud` | PointCloud2 | 카메라 optical frame; XYZ float32; m |

=== "호출·상세 규칙"

    - **ROS 호출:** 외부 서비스·액션 없음. BT 내부 CPU 연산입니다.
      `/grasp/object_cloud`에 같은 점군을 디버그용으로 발행합니다.

    - 이 단계가 실패하면 모델 추론은 실행하지 않습니다.

    **기하 처리.** 마스크가 선택한 픽셀 중 유효한 깊이를 골라 내부 파라미터로 XYZ를 복원합니다.
    개념적으로 `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy`이며, 지원하는 왜곡 보정을 함께 적용합니다.
    출력은 카메라 optical frame의 점군이므로 base 변환은 후보 선택 단계까지 미룹니다.

    **완료와 실패.** 최소 점 수를 충족하면 Blackboard 출력과 디버그 발행을 마치고 성공합니다.
    지원하지 않는 인코딩·왜곡, 잘못된 저장 크기·frame·stamp, 점 수 부족은 실패입니다.
    디버그 토픽 구독자가 없어도 계산 결과는 다음 노드로 전달됩니다.

</div>

<div class="bt-plugin" markdown="1">

### InferGrasps

<span class="bt-plugin-kind">모델 액션</span>

물체 점군을 GraspGenX에 보내 파지 자세·점수 후보를 받습니다.

!!! note "알아둘 점"

    모델 점수가 높아도 TCP 변환·충돌·손목 제약 검사에서 탈락할 수 있습니다.

=== "처리 흐름"

    1. `POINT_CLOUD` 모드, `piper_hand`와 후보 수 설정으로 goal을 만듭니다.

    2. 모델 브릿지에 점군을 보내고 결과를 기다립니다.

    3. 성공 응답, `OK:` 메시지, 비어 있지 않은 후보 배열을 확인합니다.

    4. `candidates`를 Blackboard에 출력합니다.

    **완료 조건**

    모델 파지 후보를 얻었습니다. 실행 가능성은 `SelectFeasibleGrasp`에서 검사합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `cloud` | 필수 | 물체 XYZ PointCloud2 |
    | 입력 | `server_name` | infer_grasps | Nav2 액션 서버 이름 |
    | 입력 | `server_timeout` | 기본 XML 1000ms | goal 요청 응답 대기; 추론 전체 제한과 별개 |
    | 출력 | `candidates` | GraspCandidate[] | 입력 점군 frame/stamp의 모델 자세·점수 |

=== "호출·상세 규칙"

    - **직접 호출:** `/infer_grasps` (`rwh_beyond_msgs/action/InferGrasps`). BT의 결과 대기는 70초입니다.

    - 후보 좌표계·timestamp는 입력 점군을 따릅니다. 여기서는 최종 실행 TCP나 접근 궤적을 결정하지 않습니다.

    `SegmentObject`·`InferGrasps`는 Nav2 액션 클라이언트의 `server_name`·`server_timeout` 포트도 사용합니다.
    기본 XML의 `server_timeout="1000"`은 목표 요청 응답 대기 1000ms이며 모델 추론 전체 제한 시간이 아닙니다.

    **결과의 의미.** 성공은 모델 후보를 얻었다는 뜻이며 로봇의 도달 가능성을 보장하지 않습니다.
    점수가 높은 후보라도 TCP 변환·충돌·손목 제약에서 탈락할 수 있습니다.
    BT는 이 단계에서 후보 하나를 확정하지 않으며, 최종 선택은 `SelectFeasibleGrasp`가 담당합니다.

</div>

## 파지 선택·실행

<div class="bt-plugin" markdown="1">

### SelectFeasibleGrasp

<span class="bt-plugin-kind">계획 선택</span>

모델 후보를 로봇 TCP 목표로 바꾸고 실행 가능한 첫 계획을 선택합니다.

!!! note "알아둘 점"

    장면은 변경하지만 팔·그리퍼는 실행하지 않습니다. 모든 후보를 비교해 전역 최적 경로를 고르는 방식은 아닙니다.

=== "처리 흐름"

    1. 관측 나이·촬영 후 팔 이동·frame/stamp·마스크와 점군 일치를 검사합니다.

    2. 촬영 TF로 base 변환 후 물체·수평 지지면을 추정하고 대상 형상·접촉 허용을 장면에 등록합니다.

    3. 원래 후보와 180° 대칭 자세를 만들고 회전 선호도·방향·높이·폭·지지면 여유를 검사합니다.

    4. 사전 접근 IK → 접근 Cartesian → 리프트 Cartesian → 사전 접근 OMPL 순서로 검사합니다.

    5. 첫 통과 후보의 목표·궤적·손목 제약·폭·리프트 거리를 `selected_plan`에 저장합니다.

    **완료 조건**

    후속 모션에서 공유할 `GraspPlan`을 선택했습니다. 접근·리프트 경로는 실행 시 다시 계산합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `capture`, `mask`, `cloud`, `candidates` | 필수 | 동일 촬영의 관측·마스크·점군·모델 후보 |
    | 입력 | `use_estimated_width` | false | true이면 점군 기반 폭 추정 |
    | 입력 | `pregrasp_distance` | YAML; 현재 0.10m | 사전 접근 거리; 0.02~0.20m |
    | 입력 | `lift_distance` | YAML; 현재 0.08m | 리프트 거리; 0.065~0.20m |
    | 출력 | `selected_plan` | shared_ptr&lt;GraspPlan&gt; | 후속 노드의 plan 포트와 연결 |
    | 출력 | `pregrasp`, `grasp_pose` | TcpPose | base 좌표계 사전 접근·최종 TCP 목표 |
    | 출력 | `grasp_width`, `close_width` | double; m | 관측 폭·명령 폭; 고정 폭 모드의 관측 폭은 0 |

=== "호출·상세 규칙"

    1. 관측 나이 30초 이내, 촬영 후 arm 관절 변화 0.02rad 이내인지 검사합니다.
       마스크·점군·후보의 frame/stamp와 실제 마스크·깊이에서 재투영한 점군의 일치도 확인합니다.
    2. 촬영 시각 TF로 점군을 base로 변환하고 주변 깊이로 수평 지지면을 추정합니다.
       대상 AABB 박스를 등록하고 대상과 두 손가락의 접촉을 허용합니다.
       Octomap 사용 시 대상 영역을 제외하고 지도를 다시 확인합니다.
    3. 모델 자세와 180° 대칭 자세를 만들고, 설정에 따라 현재 TCP와 회전 차이가 작은 순서로 검사합니다.
    4. 하향 각도·대상 경계·선택적 높이·지지면 여유를 검사합니다.
       폭 추정 모드에서는 모델의 손가락 닫힘 축으로 점군 폭을 계산합니다.
    5. 사전 접근 IK → 접근 Cartesian → 리프트 Cartesian → 사전 접근 OMPL 경로를 검사합니다.
       손목 제약과 반환 경로의 목표 관절값도 확인하고 첫 통과 후보를 선택합니다.

    **직접 호출:** TF, `/compute_fk`, `/compute_ik`, `/compute_cartesian_path`, `/plan_kinematic_path`,
    `/get_planning_scene`, `/apply_planning_scene`.
    Octomap 사용 시 `/clear_octomap`과 sim의 `/simulation/set_grasp_region` 또는 real의 `/grasp/set_grasp_region`도 호출합니다.
    선택 중에는 팔·그리퍼를 실행하지 않습니다. 모두 탈락하면 사유별 횟수와 함께 실패합니다.

    최종 TCP는 `base_T_camera × camera_T_model_grasp × model_T_tcp`로 계산합니다.
    사전 접근은 모델 접근축 반대 방향으로 떨어진 위치이고, 리프트는 base +Z 방향입니다.
    추정 폭 모드의 닫기 폭은 관측 폭에서 `grasp_width_closing_inset`을 빼고 최대 열림 폭으로 제한합니다.
    inset 허용 범위는 0~0.005m, 현재 값은 0.005m입니다. 고정 폭 모드는 현재 `gripper_close_width=0.04m`를 사용합니다.

    **계획 객체에 저장되는 것.** 선택한 관측, 사전 접근·최종 TCP 자세, 실행할 사전 접근 궤적,
    손목 제약, 관측 폭·명령 폭, 리프트 거리를 하나의 `GraspPlan`에 담습니다.
    접근·리프트의 사전 검사 궤적은 실행 가능성 확인용이며, 실제 실행 노드에서는 현재 상태로 다시 계산합니다.
    `pregrasp`·`grasp_pose`·폭 출력은 조회·사용자 트리용이고, 후속 기본 모션 노드는 `plan` 안의 값을 사용합니다.

    **후보 탈락과 요청 실패.** `not_top_down`, `off_target`, `height`, `table_clearance`,
    `invalid_observed_width`, `pre_ik`, `wrist_goal`, `approach_path`, `lift_path`, `pre_plan`은
    후보별 탈락 집계입니다. 반면 오래된 촬영·점군 불일치·지원면 추정 실패·서비스 예외는
    선택 작업 자체를 실패시킵니다. 첫 통과 후보를 선택하므로 모든 실행 가능 후보를 평가해
    전역 최적 경로를 고르는 방식은 아닙니다. 장면·충돌 허용은 변경하지만 궤적 실행은 하지 않습니다.

</div>

<div class="bt-plugin" markdown="1">

### MoveToPregrasp

<span class="bt-plugin-kind">팔 실행</span>

선택 단계에서 저장한 궤적을 실행해 사전 접근 위치로 이동합니다.

!!! note "알아둘 점"

    선택 후 로봇을 수동 이동했다면 다시 관측·선택해야 합니다. 이 노드는 새 OMPL 계획을 만들지 않습니다.

=== "처리 흐름"

    1. 현재 선택 계획과 같은 포인터인지, 단계가 `SELECTED`인지 확인합니다.

    2. 촬영 유효성과 궤적 시작 상태를 다시 검사합니다. 시작 관절 차이가 0.20rad를 넘으면 거부합니다.

    3. 저장 궤적을 실행하고 최종 관절 피드백을 확인합니다.

    **완료 조건**

    계획 상태가 `SELECTED → PREGRASP`로 바뀝니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `plan` | 필수; SELECTED | 현재 선택된 GraspPlan 포인터 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/execute_trajectory`. 새 OMPL 계획을 만드는 단계가 아닙니다.

    - 완료 시 `PREGRASP`가 됩니다. 실제 시작 관절값이 계획과 크게 다르면 실행을 거부합니다.

    **사용 시점.** 후보 선택 이후 로봇을 수동으로 움직였다면 저장 경로를 그대로 쓰지 말고 다시 관측·선택해야 합니다.
    이 노드에 다른 목표 자세를 추가 입력해 저장 계획을 덮어쓰는 기능은 없습니다.

</div>

<div class="bt-plugin" markdown="1">

### ApproachGrasp

<span class="bt-plugin-kind">팔 실행</span>

사전 접근 위치에서 최종 파지 TCP까지 직선 접근을 계획·실행합니다.

!!! note "알아둘 점"

    부분 Cartesian 경로는 실행하지 않습니다. OMPL 우회 경로로 대체하지 않습니다.

=== "처리 흐름"

    1. 계획 단계 `PREGRASP`를 확인합니다.

    2. 최신 시작 상태에서 저장된 `grasp_pose`까지 Cartesian을 다시 계획합니다.

    3. 성공 코드·fraction ≥ 0.999·궤적 유효성·손목 제약을 검사한 뒤 실행·도달을 확인합니다.

    **완료 조건**

    계획 상태가 `PREGRASP → APPROACHED`로 바뀝니다. 아직 그리퍼는 닫지 않았습니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `plan` | 필수; PREGRASP | 현재 선택된 GraspPlan 포인터 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/compute_cartesian_path`, `/execute_trajectory`.

    - 성공 코드·경로 fraction ≥0.999·궤적 유효성·선택된 손목 제한을 요구합니다.
      완료 시 `APPROACHED`가 됩니다.

    **실패 시 동작.** 접근은 부분 Cartesian 경로를 실행하지 않고 실패합니다.
    `MovePlace`와 달리 OMPL 우회 계획으로 대체하지 않으므로 파지 접근축의 직선 동작을 유지합니다.
    그리퍼 닫기는 다음 노드이므로 이 단계의 성공만으로 대상이 부착되지 않습니다.

</div>

<div class="bt-plugin" markdown="1">

### AttachObject

<span class="bt-plugin-kind">닫기·부착</span>

선택된 폭으로 그리퍼를 닫고 대상 충돌 형상을 TCP에 부착합니다.

!!! note "알아둘 점"

    닫기 시작 후 실패하면 자동으로 열거나 형상을 제거하지 않습니다. 부착이 불확실하면 후속 이동을 거부합니다.

=== "처리 흐름"

    1. `APPROACHED`를 확인하고 닫기 전에 내부 `attached=true`를 설정합니다.

    2. 계획의 `close_width`와 YAML의 닫기 시간으로 그리퍼 액션을 실행합니다.

    3. 대상 박스를 TCP에 부착하고 손가락·TCP 접촉을 허용합니다.

    4. `release_ready=false`, 계획 단계 `ATTACHED`를 기록합니다.

    **완료 조건**

    닫기 액션과 계획 장면의 부착 등록이 끝났습니다. 실제 잡힘을 센서로 확인한 결과는 아닙니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `plan` | 필수; APPROACHED | 계획의 close_width로 닫기; 닫기 시간은 YAML, 현재 2초 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/gripper_controller/follow_joint_trajectory`, `/apply_planning_scene`.

    - 접촉 허용 링크는 `gripper_link1`, `gripper_link2`, TCP입니다. 완료 시 `ATTACHED`가 됩니다.

    - 닫기를 시작한 뒤 실패하면 잡힘 여부가 불명확하므로 자동으로 열거나 대상 형상을 제거하지 않습니다.
      부착은 계획상의 가정이며 실제 잡힘 성공을 판정하지 않습니다.

    **닫기 판정의 한계.** 닫기 명령은 개방과 달리 목표 간격에 대한 5mm 피드백 도달 검사를 하지 않습니다.
    물체 접촉으로 목표 폭에 도달하지 못할 수 있기 때문이며, 액션 성공을 잡힘 센서로 해석하면 안 됩니다.
    닫기 중단·장면 부착 실패 시에도 자동 개방하지 않고, 실제 부착이 없는데 내부 부착만 남은 경우
    후속 이동의 상태 검사에서 거부합니다.

</div>

<div class="bt-plugin" markdown="1">

### LiftObject

<span class="bt-plugin-kind">팔 실행</span>

부착된 대상을 현재 TCP 기준 base +Z 방향으로 들어 올립니다.

!!! note "알아둘 점"

    실패·취소 시에도 부착과 닫힘을 유지합니다.

=== "처리 흐름"

    1. 계획 단계 `ATTACHED`와 유지된 부착 정보를 확인합니다.

    2. 현재 TCP를 FK로 구하고 저장된 `lift_distance`만큼 base Z를 더합니다.

    3. 부착 형상을 포함해 Cartesian을 다시 계획하고 완전성·손목 제약을 검사합니다.

    4. 실행 결과와 관절 도착을 확인합니다.

    **완료 조건**

    계획 상태가 `ATTACHED → LIFTED`로 바뀝니다. home 복귀는 뒤의 XML 노드가 담당합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `plan` | 필수; ATTACHED | 계획의 lift_distance 사용; 부착 정보 유지 필요 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** `/compute_fk`, `/compute_cartesian_path`, `/execute_trajectory`.

    - 접근과 같은 완전성·손목 제한 검사를 수행합니다. 완료 시 `LIFTED`입니다.

    **완료와 실패.** 궤적 실행·관절 도착 확인 후 `LIFTED`로 바뀝니다.
    부분 경로·손목 제약 위반·실행 실패는 모두 실패이며, 실패해도 부착과 닫힘을 유지합니다.
    이 노드는 home 복귀까지 수행하지 않으므로 XML 뒤에 복귀 노드를 별도로 둡니다.

</div>

## 배치·해제

<div class="bt-plugin" markdown="1">

### MovePlace

<span class="bt-plugin-kind">배치 이동</span>

home 기준 배치 위치로 이동하고 해제 가능한 상태를 기록합니다.

!!! note "알아둘 점"

    Cartesian 요청 오류는 우회 성공으로 처리하지 않습니다. 실패하면 부착·닫힘을 유지합니다.

=== "처리 흐름"

    1. `release_ready=false`로 초기화하고 대상의 부착 상태를 확인합니다.

    2. home FK 위치에 base X·Z 오프셋을 더합니다. 방향은 home TCP 방향을 유지합니다.

    3. Cartesian이 완전하면 실행합니다. 정상 응답의 부분 경로일 때만 같은 목표의 IK·OMPL 경로를 시도합니다.

    4. 실행과 피드백 도착 확인 후 `release_ready=true`로 바꿉니다.

    **완료 조건**

    배치 이동이 끝나 `ReleaseObject`를 실행할 수 있습니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `home_joint_positions` | YAML | home 기준 arm 6축 관절값(rad) |
    | 입력 | `x` | 0m | home TCP + base X; 0~0.30m |
    | 입력 | `z` | 0m | home TCP + base Z; 0~0.30m |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** Planning Scene 확인, `/compute_fk`, `/compute_cartesian_path`,
      필요 시 `/compute_ik`, `/plan_kinematic_path`, 이후 `/execute_trajectory`.

    - **조건:** 부착된 대상이 필요합니다. 현재 자세에서 목표로 계획하며 home 도착은 요구하지 않습니다.

    - 완료 시 `release_ready=true`로 표시합니다. 실패하면 부착·닫힘 상태를 유지합니다.

    **완료와 실패.** 실행과 피드백 도착 확인까지 성공해야 `release_ready=true`가 됩니다.
    잘못된 응답·IK 실패·실행 실패는 대체 성공으로 취급하지 않으며 물체를 유지합니다.
    Y 입력은 없고, placement 요청마다 새 모델 추론을 수행하지 않습니다.

</div>

<div class="bt-plugin" markdown="1">

### ReleaseObject

<span class="bt-plugin-kind">개방·해제</span>

배치 완료를 확인한 뒤 개방·부착 해제·임시 장면 정리를 수행합니다.

!!! note "알아둘 점"

    개방 후 장면 정리가 실패하면 물리 상태와 계획 장면이 다를 수 있습니다. 일반 호출은 배치 완료 조건을 생략할 수 없습니다.

=== "처리 흐름"

    1. 실제 부착 상태와 성공한 `MovePlace`의 `release_ready`를 확인합니다.

    2. 그리퍼를 열고 액션 결과와 실제 개방 간격을 확인합니다.

    3. Planning Scene에서 부착을 제거하고 내부 부착·배치 플래그를 내립니다.

    4. 대상 형상·변경했던 충돌 허용·활성화된 점군 제외 영역을 정리합니다.

    **완료 조건**

    대상을 해제하고 미부착 상태로 돌아왔습니다. home 복귀는 다음 노드에서 수행합니다.

=== "입출력"

    | 방향 | 포트 | 기본값·타입 | 의미·범위 |
    |---|---|---|---|
    | 입력 | `opening` | YAML; 현재 0.10m | 전체 개방 간격 0~0.10m |
    | 입력 | `duration` | 3초 | 0.001~30초 |

    출력 포트는 없습니다.

=== "호출·상세 규칙"

    - **직접 호출:** Planning Scene 확인, `/gripper_controller/follow_joint_trajectory`,
      `/apply_planning_scene`, `/get_planning_scene`, Octomap 사용 시 대상 제외 서비스.

    - **조건:** 대상 부착과 성공한 `MovePlace`의 `release_ready`가 필요합니다.

    - 기본값은 `OpenGripper`와 같습니다. 임의 위치에서 물체를 놓는 범용 노드가 아닙니다.

    **실패를 해석할 때.** 개방 확인 전에 실패하면 부착 해제 단계로 진행하지 않습니다.
    개방 이후 장면 갱신·정리가 실패하면 물리 상태와 계획 장면이 다를 수 있으므로 성공으로 보고하지 않습니다.
    `CheckGraspReady release_previous=true`는 공통 release 로직의 별도 허용 경로이고,
    일반 `ReleaseObject`는 `MovePlace` 성공 조건을 생략할 수 없습니다.

</div>

## 계획 상태와 공통 동작

`selected_plan="{grasp_plan}"`을 후속 노드의 `plan="{grasp_plan}"`으로 연결합니다.
단계 상태는 `SELECTED → PREGRASP → APPROACHED → ATTACHED → LIFTED`입니다.
다른 계획·단계 생략·이미 실행한 단계의 재실행은 거부합니다. 문자열 plan_id나 서버 세션은 사용하지 않습니다.

모션은 비동기 BT 작업으로 수행하며 진행 중에는 `RUNNING`을 반환합니다.
서로 다른 BT 분기에서 동시에 같은 로봇을 명령하면 거부합니다. halt는 진행 중 작업과 액션을 취소합니다.
미부착 실패는 장면을 정리하고, 닫기 시작 후·부착 후 실패는 자동 개방하지 않습니다.

모션 설정은 `bt_navigator.ros__parameters.bt_motion`, 인식 설정은 `bt_perception`입니다.
XML 입력은 해당 호출에만 적용되고 YAML 기본값을 바꾸지 않습니다.
모션 노드에는 중계용 `server_name`·`server_timeout`이 없습니다.

## Groot와 이전 XML

노드 정의는 `node_models/rwh_beyond_tree_nodes.xml`입니다. 실행 XML과 별개이며 다음 명령으로 재생성합니다.

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml
```

기존 XML의 `SelectFeasibleGrasp.plan_id` 출력은 `selected_plan`으로,
후속 모션 노드의 `plan_id` 입력은 `plan`으로 바꿉니다.
`RunGraspStage`·`RunGraspStep` 중계 액션은 제거됐습니다. 인식 입력은 capture·mask·cloud·candidates로 직접 연결합니다.


## 구현 위치와 tick 방식

??? info "라이브러리·구현 파일 보기"

    | 라이브러리 | 노드 수 | 구현 |
    |---|---:|---|
    | `rwh_beyond_perception_bt_nodes` | 4 | `plugins/action/perception_actions.cpp` |
    | `rwh_beyond_motion_bt_nodes` | 11 | `plugins/action/`의 노드별 파일, 공통 실행은 `motion_action.cpp`, 연산·통신은 `src/motion_context.cpp` |
    | `rwh_beyond_clear_octomap_action_bt_node` | 1 | `plugins/action/clear_octomap_action.cpp` |
    | `rwh_beyond_bt_nodes` | 전체 로더 | `plugins/register_nodes.cpp`; 위 3개 라이브러리를 등록 |

    경로는 `rwh_beyond_bt/` 기준입니다. 모션 노드들은 BT 내부의 `MotionContext`를 공유합니다.
    노드별 헤더·소스와 등록 관계는 [플러그인 구조](bt-architecture.md#plugin-structure)를 참고하세요.
    별도 coordinator 프로세스나 `/grasp/*` 단계 중계 액션 서버는 없습니다.

??? info "BT 노드 타입과 tick 방식 보기"

    플러그인은 XML에 적힌 **BT 노드 타입**입니다. 예를 들어 `MoveHome`은 한 타입이지만
    집기 시작과 복귀에 서로 다른 인스턴스로 배치됩니다. `Sequence`는 프로젝트 플러그인이 아니라
    BehaviorTree.CPP의 제어 노드입니다. 한 자식이 성공해야 다음 자식을 tick하며,
    실행 중인 자식은 `RUNNING`, 실패한 자식은 전체 Sequence의 `FAILURE`로 전파됩니다.

    | 구현 방식 | 해당 노드 | tick 중 동작 |
    |---|---|---|
    | StatefulActionNode | CaptureRgbd, ClearOctomap | 시작 시 수신·요청을 준비하고 다음 tick마다 완료를 확인 |
    | Nav2 BtActionNode | SegmentObject, InferGrasps | ROS 액션 goal 전송, 결과 대기, 실패·취소 전달 |
    | SyncActionNode | BuildObjectCloud | 한 번의 tick에서 점군 계산 후 성공 또는 실패 |
    | 공통 MotionAction | 나머지 모션·장면 11개 | 시작 tick에서 입력을 읽고 비동기 작업 실행, 후속 tick에서 결과 확인 |

    `SUCCESS`는 각 노드의 완료 조건을 충족했다는 뜻입니다. 계획 성공은 팔 이동 완료와 다르고,
    장면 부착 성공은 실제 잡힘을 센서로 확인했다는 뜻과 다릅니다.
    아래 내부 순서는 각 노드가 수행하는 처리이며 XML에 따로 추가할 하위 플러그인이 아닙니다.

</div>

[실행 흐름](bt-flows.md) · [구조와 호출 관계](bt-architecture.md) · [공통 설정](real-bt.md)

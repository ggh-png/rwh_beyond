# Upstream와 구조 매핑

## 구조 참고

- URL: https://github.com/open-navigation/opennav_coverage/tree/jazzy
- 확인한 commit: `194250501889ce8896c5ee8b80a4d1317750be0e`
- 작업 저장소는 coverage 구현을 복사/대량 rename하는 대신 필요한 구조를 새로 작성했다.

| 참고 프로젝트 역할 | RWH 역할 |
| --- | --- |
| `opennav_coverage`: include/src, core shared library, executable, lifecycle component | `rwh_beyond` |
| `opennav_coverage_msgs`: 별도 rosidl interface package | `rwh_beyond_msgs` |
| `opennav_coverage_demo`: launch/params 실행 통합 | `rwh_beyond_bringup` |
| BT plugin / navigator plugin / row coverage | 현재 기능이 필요하지 않아 만들지 않음 |

Fields2Cover, coverage action/msg, row planner, coverage BT XML/plugin 의존성은 없다.
이전 `husky_navigation` 기능은 RWH로 통합했으며, RWH 패키지 이름을 사용한다.

## 전체 Nav2 파라미터 출처

- 원본: `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`
- ROS package version: `1.3.13`
- Debian package: `1.3.13-1noble.20261006.042503`
- 별도 upstream YAML은 단일 파라미터 구성 요청에 따라 제거했다. 원본은 Nav2 `1.3.13` 소스와 이전 Git 이력에서 확인할 수 있다.
- SHA-256: `299f9002d6dcfbf661e532c99f32dda1766eab3afa3ab71a52378d80e7ef956f`
- 실제 사용하는 전체 파일: `rwh_beyond_bringup/params/nav2_params.yaml`

원본 YAML의 15개 파라미터 섹션을 통합 파일에 유지했다. 추가로 sensor_config, slam_toolbox, beyond_server 설정도 같은 파일에 둔다. 실제 파일은 전체 설정을 유지한 채
Husky 토픽, base_link, Twist, footprint 및 초기 속도 제한을 한 번 반영한 구성이다.
원본의 robot_radius 대신 footprint를 사용한다. 지도 경로는 map launch 인자로 전달한다.

이는 Nav2가 제공한 기본 YAML 전체이며, 모든 플러그인의 모든 선택적 파라미터를
열거한 API 명세는 아니다. 추가 플러그인의 옵션도 실제 YAML에 직접 추가할 수 있다.

실행 중에는 설치된 Nav2 기본 YAML을 다시 읽거나 튜닝값을 다시 생성하지 않는다.
namespace placeholder, use_sim_time 및 명시적으로 선택한 cmd_vel_type만 처리한다.
따라서 repository YAML의 MPPI/DWB/새 플러그인/속도/footprint 튜닝이 보존된다.
Nav2 배포 버전을 바꿀 때 해당 버전의 원본과 통합 파일을 비교해 변경점을 검토한다.

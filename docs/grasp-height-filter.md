# 최종 파지 TCP 높이 필터

공통 YAML의 `bt_navigator.ros__parameters.bt_motion`에서 조정합니다.
현재 `grasp_height_filter_enabled=false`이며 아래는 필터를 켜는 예시입니다.

```yaml
grasp_height_filter_enabled: true
grasp_height_above_object_min_m: 0.05
grasp_height_above_object_max_m: 0.10
```

이 예시는 **모델→TCP 변환 후 최종 파지 TCP**가 물체 상단보다 base Z 방향으로
5~10cm 높은 후보만 허용합니다. 상단은 base로 변환한 분할 점군 Z의 99백분위이며
충돌 박스 padding을 포함하지 않습니다. 경계값도 허용합니다.

후보 자세를 위로 옮기지 않고 조건에 맞지 않는 후보를 제외합니다.
`ApproachGrasp`가 도달하는 닫기 직전 TCP에 적용하며 사전 접근 위치의 높이 조건은 아닙니다.
TCP 높이와 손가락 여유는 다르므로 그리퍼 형상과 TCP offset도 함께 확인합니다.

필터를 켜면 대상 경계의 수직 검사를 이 높이 조건으로 대체합니다.
수평 대상 경계·하향 각도·지지면 여유·IK·충돌 검사는 유지합니다.
허용 범위는 `0 ≤ min ≤ max ≤ 0.5m`이며, 모두 0이면 물체 상단과 같은 TCP 높이만 허용합니다.
후보가 없으면 `height` 탈락 횟수를 보고하고 이동 전에 실패합니다.
자동으로 범위를 넓히지는 않습니다. 해제는 `grasp_height_filter_enabled: false`로 설정합니다.

수정 후 BT를 재활성화합니다. [후보 탈락 진단](grasp-troubleshooting.md) · [좌표계](project-data.md)

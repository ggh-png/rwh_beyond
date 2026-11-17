# 수동 시뮬레이션 통합 검사

`sam_pick_integration.py`와 `sam_runner_pick_integration.py`는 실제 SAM 3·GraspGenX 서버와
MuJoCo를 사용하는 수동 검사입니다. 일반 pytest에 포함하지 않습니다.
독립된 0이 아닌 `ROS_DOMAIN_ID`와 시뮬레이션 환경에서만 실행합니다.

```bash
ROS_DOMAIN_ID=93 python3 tests/integration/sam_pick_integration.py
ROS_DOMAIN_ID=93 python3 tests/integration/sam_runner_pick_integration.py
```

모델·MuJoCo와 runner가 필요합니다. 모의 브릿지 검사는
`rwh_beyond_graspgenx_bridge/test/`에서 수행합니다.

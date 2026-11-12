# 로컬 Podman 모델 실행

`local_podman.sh`는 GraspGenX GPU 컨테이너와 모델 프로세스를 관리합니다.
ROS와 같은 PC에서 모델을 실행할 때 사용하는 선택 구성입니다.

1. Dockerfile과 고정한 공식 소스에 맞는 이미지를 준비합니다.
2. checkpoint와 `piper_hand` assets, cache/log 경로를 준비합니다.
3. 스크립트의 환경변수로 ROS 컨테이너 네트워크 대상과 리소스를 지정합니다.
4. 컨테이너와 서버를 시작한 뒤 health를 확인합니다.

프로젝트 ROS 컨테이너 이름은 `moma_mj_lab_1.1.2_hostcan`입니다.
스크립트의 `GRASPGENX_ROBOT_CONTAINER`를 실제 대상에 맞춥니다.
로컬 server YAML의 기본 포트는 5566이며 ROS bridge endpoint와 일치해야 합니다.

모델 소스는 이미지에 포함됩니다. 소스 변경은 이미지 재빌드와 컨테이너 재생성이 필요합니다.
모델·캐시·로그는 별도 볼륨에 보관합니다.

[서버 배포](README.md) · [ROS 런타임](../../docs/piper-runtime.md)

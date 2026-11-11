# rwh_beyond_model_utils

공통 CPU 라이브러리이며 실행 노드는 없습니다. RGB/Depth/XYZ/pose 검증과 MessagePack·NumPy ZMQ wire helper를 제공합니다.
실제 모델 요청은 SAM·GraspGenX bridge가 담당합니다. CUDA·torch를 import하지 않습니다.

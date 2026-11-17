# 문서 관리

문서는 현재 실행 코드·설정·Action 정의를 기준으로 작성합니다.
과거 변경 내역과 검증 로그는 Git 이력에서 확인하며 현재 사용 가이드와 섞지 않습니다.
문서의 “현재 값”은 저장소 YAML/XML 기준이며, 장비별 변경값이나 외부 서버의 실행 상태를 뜻하지 않습니다.
의존 저장소의 runner·로봇 모델 설정도 함께 바뀔 수 있으므로 실제 workspace 버전을 확인합니다.

```bash
python3 -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r requirements-docs.txt
mkdocs serve
```

배포 전 확인:

```bash
mkdocs build --strict
git diff --check
```

메뉴와 테마는 `mkdocs.yml`, 버전은 `requirements-docs.txt`, 스타일은
`docs/assets/stylesheets/docs.css`에서 관리합니다. Mermaid fenced block으로 구조도를 작성합니다.
수식은 `pymdownx.arithmatex`와 MathJax로 렌더링하며 `$...$`는 인라인,
별도 줄의 `$$...$$`는 블록 수식입니다. MathJax 설정은
`docs/assets/javascripts/mathjax.js`, 로딩 버전은 `mkdocs.yml`에서 관리합니다.

`main`의 문서 변경은 GitHub Actions에서 strict 빌드 후 `gh-pages`로 배포합니다.
소스는 `docs/`에서 수정하며 생성된 `site/`와 `gh-pages` 파일을 직접 편집하지 않습니다.
문서는 [홈](index.md)의 목적별 안내에서 시작해 **처음 사용하기 → 설정과 운영 → 문제 해결 →
시스템 이해와 개발 → 인터페이스 참조**로 이어집니다. 실행 가이드는 절차, 설정 문서는
값의 의미·반영 범위, 참조 문서는 필드·포트와 완료 조건을 다룹니다.

Action·BT·설정 변경 시 [실행 가이드](grasp-pipeline-guide.md), [Action 인터페이스](bt-operation.md),
[공통 BT](real-bt.md), [코드 안내](project-code-map.md)를 함께 갱신합니다.

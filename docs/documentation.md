# 문서 관리

문서는 현재 실행 코드·설정·Action 정의를 기준으로 작성합니다.
과거 변경 내역과 검증 로그는 Git 이력에서 확인하며 현재 사용 가이드와 섞지 않습니다.

```bash
python3 -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r requirements-docs.txt
mkdocs serve
```

배포 전 확인:

```bash
mkdocs build --strict
```

메뉴와 테마는 `mkdocs.yml`, 버전은 `requirements-docs.txt`, 스타일은
`docs/assets/stylesheets/docs.css`에서 관리합니다. Mermaid fenced block으로 구조도를 작성합니다.

`main`의 문서 변경은 GitHub Actions에서 strict 빌드 후 `gh-pages`로 배포합니다.
소스는 `docs/`에서 수정하며 생성된 `site/`와 `gh-pages` 파일을 직접 편집하지 않습니다.
Action·BT·설정 변경 시 [실행 가이드](grasp-pipeline-guide.md), [Action 인터페이스](bt-operation.md),
[공통 BT](real-bt.md), [코드 안내](project-code-map.md)를 함께 갱신합니다.

# 문서 관리

문서는 MkDocs로 빌드하며 GitHub Pages에서 공개합니다. ROS나 로봇 연결 없이 문서만 빌드할 수 있습니다.

```bash
python3 -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r requirements-docs.txt
mkdocs serve
```

로컬에서 `http://127.0.0.1:8000`으로 확인합니다. 배포 전 검증:

```bash
mkdocs build --strict
```

`main`에 문서 변경이 push되면 GitHub Actions가 strict 빌드 후 `gh-pages` 브랜치로 배포합니다. GitHub Pages는 해당 브랜치의 루트를 게시하도록 설정합니다. 문서 수정은 `docs/`에서 하고 생성된 `site/` 또는 `gh-pages` 파일을 직접 편집하지 않습니다.

[공식 MkDocs GitHub Pages 배포 안내](https://www.mkdocs.org/user-guide/deploying-your-docs/)

## 문서 테마

[GLIM 문서](https://koide3.github.io/glim/)와 같은 Material for MkDocs 기반의 인디고 테마를 사용합니다.
좌측에는 문서 탐색, 우측에는 현재 페이지 목차가 있으며 상단에서 검색할 수 있습니다.
좁은 화면에서는 메뉴와 목차가 접힙니다. 코드 블록에는 복사 버튼과 구문 강조를 제공합니다.

- 테마·메뉴·Markdown 기능: `mkdocs.yml`
- 버전 고정: `requirements-docs.txt`
- 한국어 줄바꿈·표·다이어그램 여백: `docs/assets/stylesheets/docs.css`

테마를 바꾼 뒤에는 기존 가상환경에서도 `pip install -r requirements-docs.txt`를 다시 실행합니다.
GitHub Actions도 같은 requirements를 설치하므로 로컬과 배포 테마를 맞출 수 있습니다.

## Mermaid 다이어그램

Markdown의 `mermaid` fenced block을 사용합니다. GitHub에서는 기본 Mermaid 지원으로,
문서 사이트에서는 Material의 SuperFences 통합으로 렌더링합니다.
이전 페이지별 custom JavaScript 렌더러는 제거했습니다.

Material은 다이어그램이 있는 페이지에서 Mermaid 모듈을 CDN으로 불러옵니다.
브라우저의 CDN 접근이 필요하며 문서 strict 빌드 자체는 브라우저 렌더링을 대신하지 않습니다.

[Material 다이어그램 설정](https://squidfunk.github.io/mkdocs-material/reference/diagrams/)

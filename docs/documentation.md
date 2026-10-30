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

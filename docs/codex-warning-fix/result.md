# Codex 경고 수정 결과

## 해결한 항목

- PATH 우선순위의 독립 CLI를 0.160.1에서 공식 안정판 0.169.0으로 업데이트했다. 같은 폴더의 code-mode-host도 같은 릴리스로 맞췄다.
- 기존 CLI에서 재현된 trusted_content_hash 4개 무시 경고는 새 CLI와 기존 데스크톱 번들의 독립 app-server에서 발생하지 않았다.
- 신뢰 해시를 삭제하거나 신뢰 검사를 우회하지 않았다. config.toml의 기존 값은 전부 보존하고 승인된 ECC 비활성화 항목만 추가했다.
- 사용자의 분야 선별 선택에 따라 ECC 64개를 비활성화했고, 최종 실제 CLI 응답에서 두 경고 모두 사라졌다. 원본 파일과 플러그인·훅은 보존했다.
- 실행 파일·설정 백업은 C:\Users\asher\.codex\backups\warning-fix-20261117-074717에 있다.

## 범위와 제한

- 수정 전 전체 런타임 스킬 목록은 공식 지원 상한인 10000토큰을 초과했다. 중간 실제 모델 응답 시험에서 55개 누락 경고가 있었고 기존 사용자 세션 로그는 전체 625개 중 60개 누락을 기록했다. 실행 환경에 따라 전체 수가 다르다.
- skills.max_context_tokens=10000은 이 모델의 기본 예산과 같아 효과가 없었다. 시험 후 원복했다.
- 최종 제외 목록은 [disabled-skills.md](disabled-skills.md)에 있다. 비활성화 중에는 Codex의 직접 스킬 호출도 제한된다. 해당 skills.config 항목을 제거하거나 enabled = true로 바꾸면 복구된다.
- 실행 중인 기존 Codex·Orca 세션은 유지했다. 새 CLI 버전을 쓰려면 해당 Codex 세션을 다시 시작해야 한다.
- Claude의 스킬 원본·활성 상태는 변경하지 않았다. Codex 목록 예산에 대한 클라이언트 전용 설정이기 때문이다. 추가 스킬 설치로 목록이 커지면 용량 경고가 재발할 수 있다.

## 검증

- 공식 릴리스 자산의 SHA-256 검증, 설치 후 codex --version 확인.
- CLI·데스크톱 각각 initialize·config/read·skills/list·thread/start 성공, 설정 경고 0개.
- 최종 양쪽 로컬 API 목록 534개 중 64개 비활성화·470개 활성화, 정확한 제외 이름과 원본 파일 보존 확인. review-agent의 API 전용 노출과 junction 경로 차이를 구분했다.
- 최종 실제 codex --no-daemon exec --strict-config --ephemeral이 OK·종료 코드 0·경고/오류 이벤트 0개로 통과했다. 데스크톱은 새 독립 서버의 설정 로딩까지 확인했으며 실행 중인 GUI는 재시작하지 않았다.

## 근거

- 공식 릴리스. https://github.com/openai/codex/releases/tag/rust-v0.169.0.
- 스킬 예산 구현. https://github.com/openai/codex/blob/rust-v0.169.0/codex-rs/ext/skills/src/render.rs.
- Notion 운영 원칙. https://www.notion.so/3d53d76f887481df98f0c35fd02ebb7b. 파일 보관·노출·실행 검증을 구분했다.

Decision candidate. 신뢰 필드 삭제보다 호환되는 CLI 업데이트를 우선한다. 승인된 플러그인 콘텐츠 신뢰를 보존하기 위해서다.

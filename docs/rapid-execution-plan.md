# RAPID 스타일을 기본 구동 경로로 적용하는 계획

## 목표와 범위

`factory_line_demo.py`를 옵션 없이 실행하면 네 로봇이 RAPID 스타일 프로그램으로 움직인다.
사용자는 `*_program()`의 목표·속도·zone·tool을 수정해 시뮬레이션 동작을 바꾼다.
기존 `--motion rapid`의 실행기와 Isaac Sim 제어 연결을 재사용한다. ABB 구문 해석기나 실물 제어기는 만들지 않는다.

## 확인된 원인

기본 인자가 `time`이라 일반 실행에서는 RAPID 프로그램을 사용하지 않았다.
RAPID 실행 경로 자체는 `RapidTask.step()` → IK/보행 정책 → 관절 목표 전송으로 이미 연결되어 있다.

## 적용 순서와 완료 기준

1. 기본 인자·도움말·README를 RAPID 우선으로 통일한다. `--motion time`은 비교용으로 보존한다.
2. 프로그램 작성부와 실행기, 물리 콜백, 관절 전송 지점에 역할·단위·호출 순서를 설명한다.
3. 실행 시 선택한 구동 방식을 출력해 실행 경로를 확인한다.
4. 기존 단독 시험과 옵션 없는 RAPID 헤드리스 40초 자체 점검을 실행한다.
5. 새 렌더링 캡처를 확인하고 결과·한계를 작업 기록에 남긴 후 커밋한다.

## 영향 범위

- 호출부와 설정. `factory_line_demo.py`의 인자 기본값, 물리 콜백 설명, 구동 로그.
- 실행기. `rapid_style.py`의 명령 해석 및 피드백 설명. 수치 알고리즘과 공개 함수는 보존한다.
- 검증. 기존 `rapid_style.py`의 7개 시험, 데모의 관절 추종·적재·보행·재생 시험을 재사용한다.
- 문서. `README.md`, `checklist.md`, `context-notes.md`. 별도 fixture나 export는 없다.
- 사용자 영향. 옵션 없는 실행의 동작 소스가 바뀐다. 기존 시간표 실행은 `--motion time`으로 사용한다.
- 기존 파일과 이전 캡처를 보존한다. 검증 시 기존 `shot_*` 파일은 별도 실행 폴더에 복사해 둔다.

## 근거

- Notion 실습 기록. https://www.notion.so/4163d76f887481588f07cd3d8ffe18c4
- Isaac Sim 6.2 관절 API. https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.prims/docs/api.html
- 로컬 소스의 `on_physics_step`, `joint_step`, `RapidTask.step`.

박스는 시각용이며 실제 접촉 파지와 충돌 회피는 이번 변경 범위에 포함하지 않는다.

---
name: rapid-commands
description: 로봇 동작을 RAPID 스타일 명령(robtarget·speed·zone·tooldata)으로만 정의하는 규칙
updated: 2026-11-22
---

## 결정
- 실행 가능한 동작 정의는 RAPID 스타일 프로그램 하나뿐이다. 시간표 구동과 `--motion` 옵션은 삭제했고 Git 이력에만 있다 (사용자 명시 2026-11-17). 이유: 사용자가 RobotStudio 사용자라 읽고 고치는 프로그램과 실제 구동 경로를 일치시킨다.
- 명령 층은 `rapid_style.py`(Isaac import 없음, 단독 시험 가능). 프로그램은 제너레이터, `yield` 한 줄이 RAPID 한 줄, `RapidTask`가 스텝마다 진행한다.
- 단위는 m, m/s. 이름만 ABB를 따른다(`v1000` = 1.0 m/s, `z50` = 0.05 m). 방향은 오일러 ZYX [deg], `rx = 180`이 ABB 쿼터니언 [0, 1, 0, 0].
- `fine`은 실제 TCP가 목표 5 mm 안에 들어올 때까지 기다린다. 없으면 로봇 도착 전에 그리퍼 신호가 나가 박스가 어긋난다.
- zone은 반경 안에 들면 다음 목표로 트는 단순 방식. ABB 포물선 코너는 없다. `MoveJ`는 없다(닫힌 형태 역기구학이 없어 끝점을 못 구함).
- `MoveBase(basetarget(x, y, rz), speed, zone)`은 RAPID에 없는 명령이지만 `MoveL` 모양으로 AMMR·H1이 같이 쓴다. H1은 목표·속도만 내놓고(`goal`) 실제 위치가 zone 안에 들어와야 넘어간다. rz는 맞추지 않는다. AMMR 팔은 `MoveAbsJ`(가장 많이 움직이는 관절 기준 [deg/s]).
- basetarget은 월드 좌표. AMMR 베이스 관절은 처음 놓은 자리(`AMMR_HOME`)가 0이다.
- 박스 파지는 `doGrip` 신호만 따른다. 물리 접촉 파지는 아니다.
- KUKA는 컨베이어 트래킹이 아니다. `WaitUntil(box_at_scan)` 뒤 `vBelt`로 직선 이동할 뿐이다.
- 목표 좌표는 클릭 티칭(`snap_top`, 선택한 물체 윗면 가운데를 `robtarget(...)`으로 출력)으로 찍어 붙인다. 꼭짓점·모서리 스냅은 만들지 않았다.

## 이력
- 이전: 시간표 방식 기본, `--motion rapid` 선택 (2026-11-12) → RAPID 기본 (2026-11-17) → RAPID 전용 (2026-11-17 후속)

출처: log/2026-11.md `2026-11-12 결정`, `2026-11-17 RAPID 기본 구동 적용`, `2026-11-17 RAPID 전용 정리`

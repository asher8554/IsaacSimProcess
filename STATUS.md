<!-- 프로젝트의 현재 상태만 담는 Hot 층 파일 (3 KB 이하, 덮어쓰기) -->
# STATUS: active

## 지금 상태
- Isaac Sim 6.2 공장 라인 예시. 로봇 4대(KUKA·Kawasaki·H1·AMMR)를 RAPID 스타일 프로그램으로 움직인다.
- 2026-11-17 RAPID 프로그램을 ROS 2 노드 `/rapid_controller`로 분리했다. Isaac 쪽은 `/isaac_sim` 노드다.
- 2026-11-22 검토 수정까지 끝났다. 헤드리스·GUI 40초 자체 점검 15/15 PASS, 단위 시험 7개 통과.
- 2026-11-23 60초 실행에서 지령 지연 회귀를 찾아 고쳤다. 헤드리스 60초 15/15 PASS, 0.57x. 강제 종료 시 컨트롤러도 끝나는 것 확인.

## 다음 행동
1. Notion 보강 여부를 사용자에게 확인한다.

## 막힌 것 / 사용자 결정 필요
- 없음.

## 확인 필요
- checklist.md "확인하지 않은 것" 목록.

## 링크
- 결정: [decisions/00_index.md](decisions/00_index.md)
- 일지: [log/](log/2026-11.md)
- 할 일: [checklist.md](checklist.md)

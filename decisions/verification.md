---
name: verification
description: 완료 판정에 쓰는 자체 점검 구성과 속도 측정 규칙
updated: 2026-11-22
---

## 결정
- 완료 판정은 `python.bat factory_line_demo.py --headless --seconds 40`의 `SELF-CHECK PASS`와 캡처(`out/shot_*.png`) 직접 확인. 단위 시험은 `python.bat -m unittest discover -s tests`, 명령 층은 `python.bat rapid_style.py`.
- 자체 점검에 직립(H1 골반 > 0.8 m)과 정지 후 재생을 넣는다. 관절 각도만 보다가 넘어진 것을 놓친 RB-Y1 경험 때문이다.
- 지령이 하나도 없으면 예외 없이 FAIL로 보고해야 한다.
- 실시간 비율은 같은 날·같은 조건에서 HEAD와 비교한다. 측정 전에 다른 `kit.exe`·RL 학습 등 CPU/GPU 작업이 없는지 본다(겹치면 0.07x까지 떨어졌다). 다른 날 수치(0.72x)와 직접 비교하지 않는다.

출처: log/2026-11.md `2026-11-12 결정`, `2026-11-22 검토 수정`

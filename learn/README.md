# Isaac Sim 학습서

빈 Isaac Sim에서 시작해 외부 로봇 CAD(ABB GoFa)를 들여오고, 상위 폴더의 `factory_line_demo.py` 공장 라인을 만들어 제어하기까지 14단계로 따라 하는 HTML 문서다.

## 보기

`index.html`을 브라우저로 연다. 차트는 cdnjs의 Chart.js 4.4.1을 쓰므로 인터넷이 필요하다. 진행 체크는 브라우저 localStorage에 남는다.

## 파일

| 경로 | 내용 |
| --- | --- |
| `index.html`, `data.js` | 완성된 문서와 차트 수치. `build.py`가 만든다 |
| `template.html` | 문서 원본. `{{code:…}}`·`{{snippet:…}}`·`{{repo:…}}` 자리에 코드가 들어간다 |
| `build.py` | 템플릿에 `steps/` 코드·저장소 코드·`logs/` 수치를 채운다 |
| `steps/` | 단계별 실행 스크립트(2~8단계)와 장면 함수 `my_scene.py` |
| `logs/` | 2026-11-23 헤드리스 실행 로그. 차트 수치의 출처 |
| `cycle_data.py`, `cycle.json` | Kawasaki 적재 프로그램 한 사이클을 오프라인으로 돌린 시계열 (11단계 차트) |
| `img/` | 실행 캡처와 GoFa 리깅 실습 캡처 |
| `fonts/` | Pretendard 부분 글꼴과 라이선스(SIL OFL 1.1) |

## 다시 만들기

```powershell
# 단계 스크립트 실행 (예: 2단계). 로그를 logs/에 저장하면 차트가 바뀐다
C:\isaacsim-6.2\python.bat steps\step02_physics.py --headless --frames 45 > logs\log_step02_physics.txt
# 사이클 시계열 (저장소 루트의 rapid_controller를 쓴다)
C:\isaacsim-6.2\python.bat cycle_data.py
# 문서 생성 (표준 라이브러리만 쓴다)
python build.py
```

`step04_gofa.py`의 GoFa USD 기본 경로는 작성자 PC 경로다. `--usd` 인자로 자기 파일을 준다.

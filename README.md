# Isaac Sim 공장 자동화 라인 모션 예시

Isaac Sim 6.2에서 산업용 로봇 2대, 휴머노이드 1대, AMMR 1대가 한 라인에서 움직이는 예시다. ROS 2는 쓰지 않는다.

![박스를 집는 장면](docs/factory_line_pick.png)
![팔레트에 놓는 장면](docs/factory_line_place.png)

## 구성

| 역할 | 로봇 | 동작 | 구동 방식 |
| --- | --- | --- | --- |
| 산업용 로봇 A | KUKA KR210 L150 | 컨베이어 위 박스를 벨트 속도로 따라가며 검사 | 미분 IK |
| 산업용 로봇 B | Kawasaki RS080N + RG2 | 벨트 끝에서 박스를 들어 팔레트 3칸에 차례로 적재 | 미분 IK |
| 휴머노이드 | Unitree H1 | 라인 옆 통로를 사각형으로 순찰 보행 | Isaac Sim 내장 보행 정책 |
| AMMR | Clearpath Ridgeback + Franka | 적재 팔레트와 출하 버퍼 사이를 왕복하고 팔을 뻗음 | 관절 키프레임 |

택트 타임은 8초다. 박스는 0.225 m/s로 흐르고 8초마다 1개씩 적재된다. AMMR 왕복 주기는 40초다.

## 실행

```bash
C:\isaacsim-6.2\python.bat factory_line_demo.py
```

창이 뜨고 바로 재생된다. 창을 닫으면 끝난다. 정지(■) 후 다시 재생하면 처음부터 시작한다. 일시정지 후 재생하면 멈춘 곳에서 이어진다.

**첫 실행은 오래 걸린다.** 창고·로봇 에셋 약 1 GB를 인터넷에서 받는다. 이 PC에서 약 12분 걸렸다. 그 뒤로는 캐시를 써서 약 30초 만에 뜬다.

## 동작을 적는 두 가지 방식

네 로봇의 동작을 두 방식으로 적어 두었다. `--motion`으로 고른다. 장면과 박스는 같다.
아래 비교표는 Kawasaki 적재를 예로 들었다.

```bash
C:\isaacsim-6.2\python.bat factory_line_demo.py --motion rapid
```

| | 시간표 방식 (기본) | RAPID 방식 (`--motion rapid`) |
| --- | --- | --- |
| 코드 | `palletize_target` | `palletize_program` + `rapid_style.py` |
| 적는 것 | 몇 초에 손끝이 어디 있는가 | 어디로(robtarget), 속도(v1000), zone(z50·fine), tool(tooldata) |
| 걸리는 시간 | 시간표에 적은 그대로 | 거리와 속도로 정해진다 |
| 박스와 맞추는 법 | 박스와 같은 시계를 쓴다 | `WaitUntil(box_at_pick)`으로 박스를 기다린다 |
| 박스를 쥐고 놓는 때 | 정해진 시각 | `SetDO("doGrip", 1)`과 `SetDO("doGrip", 0)` |
| 맞추는 방향 | 공구 축이 아래를 보게만 | (rx, ry, rz) 전체 |

RAPID 방식은 ABB와 단위가 다르다. 길이는 m, 속도는 m/s다. `v1000`은 1.0 m/s, `z50`은 0.05 m다.
방향은 오일러 각 (rx, ry, rz) [deg]로 적고 `rx = 180`이 공구가 바닥을 보는 자세다 (ABB 쿼터니언 [0, 1, 0, 0]).
`MoveJ`, `wobj`는 없다. zone은 포물선으로 깎지 않고 반경 안에 들어오면 다음 목표로 방향을 튼다.

`rapid_style.py`는 Isaac Sim 없이 따로 시험할 수 있다.

```bash
C:\isaacsim-6.2\python.bat rapid_style.py
```

RAPID 방식에서는 네 로봇이 모두 프로그램으로 움직인다. 명령의 모양은 같다 (목표, 속도, zone).

| 로봇 | 시간표 방식 | RAPID 방식 프로그램 | 쓰는 명령 |
| --- | --- | --- | --- |
| KUKA | `scan_target` | `scan_program` | `MoveL`, `WaitUntil` |
| Kawasaki | `palletize_target` | `palletize_program` | `MoveL`, `WaitUntil`, `SetDO`, `WaitTime` |
| AMMR | `AMMR_KEYS` | `ammr_program` | `MoveBase`(몸체), `MoveAbsJ`(팔), `WaitTime` |
| H1 | `H1_WAYPOINTS` | `patrol_program` | `MoveBase`(걷기) |

- `MoveBase(basetarget(x, y, rz), v500, fine)`는 RAPID에 없는 명령이다. 이동 로봇의 몸체를 바닥 위 (x, y) [m]와 방향 rz [deg]로 보낸다.
  AMMR은 직선으로 가면서 방향도 맞춘다. H1은 목표 쪽으로 걸어가고 도착 방향은 맞추지 않는다.
- `MoveAbsJ(jointtarget(...), 45.0, fine)`의 속도는 [deg/s] 가장 많이 움직이는 관절의 속도다.
- KUKA는 어느 방식에서도 컨베이어 트래킹이 아니다. 박스나 벨트의 위치를 읽지 않는다.
  시간표 방식은 같은 시계로 시각만 맞추고, RAPID 방식은 박스가 시작 위치에 온 신호를 받아 벨트와 같은 속도로 직선 이동한다.

## 좌표를 찍는 방법 (티칭)

RobotStudio에서 물체 윗면에 스냅을 걸고 클릭해 타깃을 찍는 것과 같은 일을 세 가지로 할 수 있다.

1. **실행 중에 물체를 클릭한다.** 뷰포트나 Stage 창에서 물체를 클릭하면 실행한 터미널에 아래 줄이 찍힌다.
   그 줄을 복사해 코드의 목표 정의에 붙인다. 찍힌 자리에는 빨간 표시가 생긴다.

   ```
   [TEACH] robtarget(3.202, -1.499, 0.143, 180.0, 0.0, 0.0)  # /World/PalletOut 윗면 가운데
   ```

   움직이는 박스는 일시정지(Ⅱ)한 뒤 클릭하면 그 순간의 위치가 찍힌다. 같은 줄이 Isaac Sim 창의 Console 탭에도 나온다.
2. **코드에서 `snap_top("/World/물체경로")`를 쓴다.** 물체 윗면 가운데를 바로 robtarget으로 받는다.
   `pPallet = Offs(snap_top("/World/PalletOut"), 0.0, 0.0, BOX)`가 그 예다.
3. **숫자를 직접 적는다.** 벨트 끝처럼 물체가 아니라 라인에서 정한 위치일 때 쓴다. `pPick`이 그 예다.

찍히는 점은 물체를 감싸는 상자(바운딩 박스)의 윗면 중심이다. 꼭짓점이나 모서리 중점 같은 스냅은 없다.
그런 점이 필요하면 Isaac Sim의 Measure 도구를 쓸 수 있다. Window > Extensions에서 `Measure`를 켜면 Tools > Measure가 생긴다.
설치본에 확장이 들어 있는 것만 확인했고 직접 써 보지는 않았다. 이 도구는 좌표가 아니라 두 점 사이 거리를 보여 준다.

## 자체 점검

```bash
C:\isaacsim-6.2\python.bat factory_line_demo.py --headless --seconds 40
```

40초를 돌린 뒤 아래를 검사하고 `SELF-CHECK PASS` 또는 `FAIL`을 출력한다. 화면 캡처는 `out/`에 저장된다.

- 두 산업용 로봇의 손끝 평균 오차 0.03 m 미만, 공구가 아래를 향함
- AMMR 베이스가 경로를 0.3 m 이내로 따라감
- H1 골반 높이 0.8 m 초과(넘어지지 않음), 평균 0.25 m/s 넘게 보행
- 정지 후 다시 재생한 뒤 10초 동안 H1이 서 있고 두 팔이 목표를 따라감

`--headless`를 빼면 같은 점검을 GUI 창에서 한다. `--motion rapid`를 붙이면 RAPID 방식으로 점검하고 아래도 본다.

- Kawasaki가 박스를 제때 집어 팔레트 칸 중심 0.01 m 이내에 놓았는가
- KUKA가 박스 위를 나란히 갈 때 박스와의 x 차이가 0.03 m 미만인가

두 방식 모두 팔레트를 선택했을 때 클릭 티칭이 그 윗면 가운데를 보고하는지도 본다.

## 바꿔 보기

모두 `factory_line_demo.py` 위쪽 상수다.

- `TAKT`, `BELT_V` — 라인 속도. `X_OUT - X_IN = 3 * TAKT * BELT_V`를 유지해야 박스가 집히는 순간과 맞는다.
- `scan_target`, `palletize_target` — 두 산업용 로봇의 손끝 경로 (시간표 방식).
- `pPick`, `pPallet`, `tGripper`, `palletize_program` — Kawasaki의 목표 좌표·공구·프로그램 (RAPID 방식).
- `pScanStart`, `pScanEnd`, `tScanner`, `scan_program` — KUKA의 목표 좌표·공구·프로그램 (RAPID 방식).
- `bOut`, `bBuf`, `jTuck`, `jReach`, `ammr_program` — AMMR의 몸체 목표·팔 관절 목표·프로그램 (RAPID 방식).
- `patrol_program` — H1 순찰 프로그램 (RAPID 방식).
- `AMMR_KEYS` — AMMR 베이스 위치·방향과 팔 자세 키프레임.
- `H1_WAYPOINTS` — 휴머노이드 순찰 지점.

## 제한

- 박스는 물리가 없는 시각용 큐브다. 그리퍼가 박스를 실제로 쥐지 않고, 박스가 그리퍼 위치를 따라간다.
- 로봇끼리의 충돌 회피는 없다. 경로가 겹치지 않게 배치했을 뿐이다.
- 시뮬레이션은 실시간의 약 0.7배로 돈다 (RTX 3070 8 GB 기준).
- 인터넷이 없으면 에셋을 못 받아 실행되지 않는다.

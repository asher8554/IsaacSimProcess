---
name: scene-and-robots
description: 실행 형태, Isaac Sim API, 로봇 선정·배치, 물리 설정과 실측값
updated: 2026-11-22
---

## 결정
- standalone 스크립트 하나(`python.bat factory_line_demo.py`). Script Editor 버전은 없다. H1 보행 정책이 물리 콜백과 200 Hz 설정을 스크립트가 쥐어야 한다.
- API는 `isaacsim.core.experimental.*`. 6.2에서 `isaacsim.core.api`(World)는 deprecated다.
- 로봇은 에셋 서버에 실제로 있는 것만. KUKA KR210 L150, Kawasaki RS080N(OnRobot RG2), Unitree H1(내장 `RobotPolicyRunner` 보행), Clearpath Ridgeback + Franka(AMMR).
- 두 산업용 팔은 미분 IK(감쇠 최소제곱), AMMR은 관절 목표. Kawasaki 관절 부호를 손으로 맞추는 것보다 IK가 빨랐다.
- Kawasaki는 컨베이어 끝 축선 (3.2, 0). 옆 (2.2, −1.5)에서는 집기→놓기 직선이 로봇 축 0.5 m 안을 지났다.
- 물리는 CPU, 200 Hz(H1 정책 학습 조건). 8 GB VRAM은 렌더링에 남긴다.
- 박스는 물리 없는 시각용 큐브.

## 확인한 사실 (2026-11-12, Isaac Sim 6.2.0-rc.14, RTX 3070 8 GB)
- 에셋 루트 `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2`. 로컬 에셋 팩 없음, 인터넷 필요. 첫 로딩 약 12분(930 MB), 이후 캐시 `%LOCALAPPDATA%\ov\cache\hub`.
- `ConveyorBelt_A05` 길이 2.0 m, 롤러 윗면 0.77 m, 롤러에만 충돌체. `pallet.usd` 1.21 × 0.80 × 0.143 m.
- KUKA `tool0` 공구 축은 로컬 +X, Kawasaki `gripper_center`는 +Z.
- Kawasaki j2 양수 앞으로 숙임, j3·j5 양수 뒤로 젖힘, j1 양수 시계 방향.
- KUKA 기본 게인 5.7e16에서 지령 없이 a1이 0.13 rad 흐름. 5.73e8 / 5.73e6으로 낮춤(원인 분리 실험은 안 함).

출처: log/2026-11.md `2026-11-12 결정`, `2026-11-12 확인한 사실`

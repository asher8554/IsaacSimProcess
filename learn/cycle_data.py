# Kawasaki 적재 프로그램을 이상적 추종(실제 TCP = 지령)으로 돌려 명령 구간·속도·높이 시계열을 뽑는다
import json, os, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # 저장소 루트의 rapid_controller 등을 쓴다
import rapid_controller as rc
from rapid_style import RapidTask
from line_common import TOOLS, TAKT
tool = TOOLS["tGripper"]
clock = {"t": -2.0}
segs = []
def traced():
    for i, c in enumerate(rc.palletize_program()):
        segs.append([round(clock["t"], 3), i % 11])
        yield c
task = RapidTask(traced(), rc.io)
# 시작 TCP: 집는 점 45 cm 위
p0 = rc.pPick["pos"] + [0, 0, 0.45]; r0 = rc.pPick["rot"] * tool["rot"].inv()
link_pos, link_rot = p0 - r0.apply(tool["pos"]), r0
dt, t = 0.005, -2.0
picked = 0
rows = []
names = ["Pick 위 45cm (v1500,z50)","Pick (v1000,fine)","WaitUntil 박스","SetDO 1","WaitTime 0.3","들어올림 (v1000,z50)","칸 위 60cm (v1500,z50)","칸 (v1000,fine)","SetDO 0","WaitTime 0.3","빈손 올림 (v1000,z50)"]
idx = -1; last_cmd = None
while t < 30.0:
    arrived = int(np.floor(t / TAKT)) + 1 if t >= 0 else 0
    rc.di["box_at_pick"] = arrived > picked
    clock["t"] = t
    g0 = rc.io["doGrip"]
    task.step(dt, tcp=(link_pos, link_rot))
    if rc.io["doGrip"] == 1 and g0 == 0: picked += 1
    if task.pos is not None:
        tcp = task.pos
        link_pos, link_rot = tcp - (task.rot * tool["rot"].inv()).apply(tool["pos"]), task.rot * tool["rot"].inv()
    rows.append([round(t,3), round(task.speed,3), round(float(task.pos[2]) if task.pos is not None else float(p0[2]),3)])
    t += dt
print(json.dumps({"segs": segs[:40], "n": len(rows)}))
json.dump({"rows": rows[::10], "segs": segs, "names": names}, open(os.path.join(HERE, "cycle.json"), "w"))

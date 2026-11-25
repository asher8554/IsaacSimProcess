# (5단계) 장면에 로봇 3대를 놓고 Articulation으로 잡아 관절·링크 이름을 확인하는 스크립트
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument("--frames", type=int, default=0, help="이 프레임 수만큼 돌고 끝낸다. 0이면 창을 닫을 때까지")
parser.add_argument("--shot", default="", help="끝나기 직전 화면을 저장할 PNG 경로")
args = parser.parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import omni.timeline
from my_scene import build_scene  # SimulationApp을 만든 뒤에 import한다
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

scene = build_scene(simulation_app)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

# 내 로봇을 넣을 때 가장 먼저 하는 일. 관절 이름(순서)과 링크 이름을 출력해 손끝 링크를 고른다.
for name in ("kuka", "kawasaki", "ammr"):
    robot = scene[name]
    robot.reset_to_default_state()  # set_default_state로 정한 자세로 한 번에 옮긴다
    print(f"ROBOT {name} dof {robot.num_dofs}: {list(robot.dof_names)}", flush=True)
    print(f"LINKS {name}: {list(robot.link_names)}", flush=True)

frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if args.shot and frame == max(args.frames - 5, 1):
        capture_viewport_to_file(get_active_viewport(), args.shot)
    if args.frames and frame >= args.frames:
        break
print("STEP05 OK", flush=True)
timeline.stop()
simulation_app.close()

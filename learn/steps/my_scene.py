# (5단계부터) 공장 라인 장면과 로봇 3대를 만드는 함수 모음. SimulationApp을 만든 뒤에 import한다
import math

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

H = math.pi / 2  # [rad] 90도
KUKA_HOME = [0.0, 0.0, 0.3, 0.0, 1.27, 0.0]  # [rad] a1~a6. 공구가 아래를 보는 자세
KAWASAKI_HOME = [0.0, 0.2, -1.5, 0.0, -1.44, 0.0] + [0.0] * 6  # [rad] j1~j6 + 그리퍼 관절 6개
TUCK = [0.0, -1.0, 0.0, -2.4, 0.0, 1.6, 0.785]  # [rad] AMMR 팔 7축 접은 자세


def add(url, prim_path, position=(0.0, 0.0, 0.0), yaw=0.0):
    """USD 파일 하나를 참조로 불러와 [m] position, [rad] yaw(위에서 본 반시계 회전)에 놓는다.

    url이 "/Isaac/..."로 시작하면 NVIDIA 에셋 서버의 파일이고, "E:/..."처럼 드라이브 문자로 시작하면 내 PC의 파일이다.
    """
    full = url if url[1:2] == ":" else get_assets_root_path() + url
    stage_utils.add_reference_to_stage(full, prim_path)
    XformPrim(prim_path, positions=list(position), orientations=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], reset_xform_op_properties=True)


def build_scene(simulation_app, with_robots=True):
    """빈 장면 → 물리 → 창고·컨베이어·팔레트·박스 → (로봇 3대)를 만들고 다 읽을 때까지 기다린다."""
    if get_assets_root_path() is None:
        raise RuntimeError("에셋 서버에 연결할 수 없다. 인터넷 연결을 확인한다.")
    stage_utils.create_new_stage()
    stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
    SimulationManager.set_physics_sim_device("cpu")
    SimulationManager.set_physics_dt(1 / 200)  # [s] 물리 0.005 s
    RenderingManager.set_dt(8 / 200)  # [s] 화면 간격 요청값 (2단계에서 잰 실제는 update 1번에 물리 6스텝)
    simulation_app.update()

    add("/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/World/Warehouse")
    for i in range(3):
        add("/Isaac/Props/Conveyors/ConveyorBelt_A05.usd", f"/World/Conveyor_{i}", (-4.0 + 2.0 * i, 0.0, 0.0))
    add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletOut", (3.2, -1.5, 0.0))
    add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletBuffer", (-3.0, -1.5, 0.0))
    boxes = Cube([f"/World/Boxes/box_{i}" for i in range(6)], sizes=0.35, colors=[0.72, 0.52, 0.30],
                 positions=[[-3.7 + 1.08 * i, 0.0, 0.77 + 0.175] for i in range(6)])
    scene = {"boxes": boxes}
    if with_robots:
        # (x, y, z)는 [m] 로봇 베이스를 놓는 곳, yaw는 [rad] 바라보는 방향
        add("/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd", "/World/Kuka", (-1.0, 2.2, 0.0), yaw=-H)
        add("/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd", "/World/Kawasaki", (3.2, 0.0, 0.0), yaw=-1.5 * H)
        add("/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd", "/World/AMMR", (3.2, -3.0, 0.0))
        # add()는 모양을 놓기만 한다. 관절을 읽고 움직이려면 Articulation(경로)으로 잡는다
        scene["kuka"] = Articulation("/World/Kuka")
        scene["kawasaki"] = Articulation("/World/Kawasaki")
        scene["ammr"] = Articulation("/World/AMMR")
        # 재생을 시작할 때 돌아갈 자세. 값 개수 = 그 로봇의 전체 관절 수
        scene["kuka"].set_default_state(dof_positions=KUKA_HOME)
        scene["kawasaki"].set_default_state(dof_positions=KAWASAKI_HOME)
        scene["ammr"].set_default_state(dof_positions=[0.0, 0.0, 0.0, *TUCK, 0.04, 0.04])
    while stage_utils.is_stage_loading():
        simulation_app.update()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[8.5, -8.0, 6.0], target=[0.0, 0.5, 0.8])
    return scene

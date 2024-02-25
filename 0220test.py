import argparse     #解析命令行参数
from omni.isaac.kit import SimulationApp
import numpy as np      #数值计算
import omni             #访问omniverse的工具

# This sample loads a usd stage and starts simulation，定义CONFIG词典
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}


# Set up command line arguments
parser = argparse.ArgumentParser("Usd Load sample")
parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")
#添加--headless参数

args, unknown = parser.parse_known_args()       #用于处理额外参数
# Start the omniverse application
CONFIG["headless"] = args.headless
simulation_app = SimulationApp(launch_config=CONFIG)

from omni.isaac.core import World           #表示场景世界
from omni.isaac.core.robots import Robot    #表示机器人
from omni.isaac.core.utils.types import ArticulationAction  #表示关节动作


omni.usd.get_context().open_stage("/home/yangfan/test/0220.usd")

# wait two frames so that stage starts loading，更新模拟应用程序状态
simulation_app.update()
simulation_app.update()

#打印提示消息并导入函数检查场景是否正在加载
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

world = World(stage_units_in_meters=1.0)    #创建了一个名为 world 的 World 实例，设置单位长度为1米
robot = world.scene.add(Robot(prim_path="/World/flexiv_robot", name="robot"))  

world.reset()

while simulation_app.is_running():
    world.step(render=not args.headless)

    # deal with pause/stop
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()

        # apply actions，为机器人应用一个随机的关节动作。
        robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=np.random.random(7))
        )

simulation_app.close()

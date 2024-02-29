# test
导入仿真软件  
``` 
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }    //headless即无GUI模式
)
```
打开名为"simple_case.usd"的USD场景文件
``omni.usd.get_context().open_stage("simple_case.usd")``

**motion_gen_reacher.py**

张量操作
```
import torch
a = torch.zeros(4, device="cuda:0")
```
argparse模块用于解析命令行参数,从命令行获取程序运行时的参数设置

通过ArgumentParser类创建了一个解析器对象parser，然后定义了一系列命令行参数。  
--robot: 用于指定要加载的机器人配置文件的名称，默认为 "franka.yml"。  
--external_asset_path: 用于指定加载外部机器人时的外部资源路径。默认为 None，即不指定外部路径。  
--external_robot_configs_path: 用于指定加载外部机器人时的外部机器人配置文件路径。默认为 None，即不指定外部路径。
--visualize_spheres: 当设置为 True 时，可视化机器人的球形部件。该参数采用布尔值。  
--reactive: 当设置为 True 时，以反应模式（reactive mode）运行。同样采用布尔值。
```
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)
parser.add_argument("--robot", type=str, default="franka.yml", help="robot configuration to load")
parser.add_argument(
    "--external_asset_path",
    type=str,
    default=None,
    help="Path to external assets when loading an externally located robot",
)
parser.add_argument(
    "--external_robot_configs_path",
    type=str,
    default=None,
    help="Path to external robot config when loading an external robot",
)

parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)
parser.add_argument(
    "--reactive",
    action="store_true",
    help="When True, runs in reactive mode",
    default=False,
)
args = parser.parse_args()
```

导入仿真，创建了一个 SimulationApp 的实例对象。在创建实例时，传入了一个字典作为参数，字典中包含仿真的配置信息
```
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)
```
从Python的 typing 模块中导入了 Dict 类型，用于指定字典类型的变量  
carb模块日志记录；numpy库重命名为 np，用于进行数值计算和数组操作；导入World类，用于创建仿真世界；导入了 cuboid 和 sphere 类，用于创建立方体和球形物体。  
从自定义的 helper 模块（example文件夹内）中导入了两个函数 add_extensions 和 add_robot_to_scene，用于向场景中添加扩展和机器人。
```
from typing import Dict

# Third Party
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere
```

cuRobo，提供运动规划、碰撞检测等机器人运动规划相关的功能
```
from omni.isaac.core.utils.types import ArticulationAction

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.logger import log_error, setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
```

```
def main():
    # create a curobo motion gen instance:

    # assuming obstacles are in objects_path:
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage

    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    stage.DefinePrim("/curobo", "Xform")
    # my_world.stage.SetDefaultPrim(my_world.stage.GetPrimAtPath("/World"))
    stage = my_world.stage
    # stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
```
定义了障碍物立方体和障碍物网格的数量.障碍物的位置，用一个长度为3的NumPy数组表示，分别表示障碍物在 x、y、z 方向上的位置。障碍物的位置是 [0.5, 0, 0.5]，意味着它位于场景中的 x 和 z 方向的中心，y 方向为地面。  
orientation是障碍物的方向，数组表示障碍物的四元数方向。在这里，四元数表示 [0, 1, 0, 0]，这可能表示障碍物沿着 y 轴方向朝上，没有旋转。  
size=0.05:障碍物的大小，表示立方体的边长
```
    target = cuboid.VisualCuboid(
        "/World/target",
        position=np.array([0.5, 0, 0.5]),
        orientation=np.array([0, 1, 0, 0]),
        color=np.array([1.0, 0, 0]),
        size=0.05,
    )

    setup_curobo_logger("warn")
    past_pose = None
    n_obstacle_cuboids = 30
    n_obstacle_mesh = 100
```

warm up实例  
robot_cfg_path = get_robot_configs_path(): 调用了函数，获取了机器人配置文件的路径，并将其存储在变量中。
如果**命令行参数**中指定了外部机器人配置文件的路径 args.external_robot_configs_path，则将 robot_cfg_path 更新为该路径。  
robot_cfg = load_yaml(join_path(robot_cfg_path, args.robot))["robot_cfg"]: 加载了机器人配置文件，并从中获取了机器人配置信息，并将其存储在变量 robot_cfg 中。  
外部机器人配置文件的路径 args.external_robot_configs_path；获取机器人配置中关节名称和默认配置信息，并将其存储在 j_names 和 default_config 变量中。 
调用函数 add_robot_to_scene(robot_cfg, my_world)，向场景中添加了机器人，并返回了机器人对象和其在场景中的路径。  
获取了世界配置文件中的碰撞表格信息，并更新了其中一个立方体的高度；获取了世界配置文件中的网格信息，并更新了其中一个网格的名称和高度。  
创建了一个 WorldConfig 的实例对象 world_cfg，其中包含了立方体和网格的配置信息。
```
    usd_help = UsdHelper()
    target_pose = None

    tensor_args = TensorDeviceType()
    robot_cfg_path = get_robot_configs_path()
    if args.external_robot_configs_path is not None:        # 检查是否指定了外部机器人配置文件的路径
        robot_cfg_path = args.external_robot_configs_path
    robot_cfg = load_yaml(join_path(robot_cfg_path, args.robot))["robot_cfg"]

    if args.external_asset_path is not None:
        robot_cfg["kinematics"]["external_asset_path"] = args.external_asset_path
    if args.external_robot_configs_path is not None:
        robot_cfg["kinematics"]["external_robot_configs_path"] = args.external_robot_configs_path
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)

    articulation_controller = robot.get_articulation_controller()

    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    world_cfg_table.cuboid[0].pose[2] -= 0.04    # 将场景中立方体障碍物的位置沿着 z 轴方向下移了 0.04 个单位
    world_cfg1 = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    ).get_mesh_world()        # 网格障碍物的配置信息，并将其保存在 world_cfg1 变量中，get_mesh_world() 可能是从加载的配置文件中获取网格障碍物的函数
    world_cfg1.mesh[0].name += "_mesh"    # 给场景中的第一个网格障碍物的名称末尾加上了 "_mesh" 后缀
    world_cfg1.mesh[0].pose[2] = -10.5    # 将场景中的第一个网格障碍物的位置沿着 z 轴方向设置为 -10.5

    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)    # 整个场景的配置信息
```
在响应式模式下对运动规划的一些参数进行调整。  
使用上述参数配置了一个运动生成器的配置对象motion_gen_config，将作为参数传递给运动生成器的构造函数，用于初始化运动生成器。  
创建了一个运动生成器对象 motion_gen，并使用配置对象 motion_gen_config 初始化了这个对象，打印提示消息：运动生成器已经准备就绪
```
    trajopt_dt = None
    optimize_dt = True
    trajopt_tsteps = 32
    trim_steps = None
    max_attempts = 4
    if args.reactive:
        trajopt_tsteps = 36
        trajopt_dt = 0.05
        optimize_dt = False
        max_attemtps = 1
        trim_steps = [1, None]
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12,
        num_graph_seeds=12,
        interpolation_dt=0.05,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        optimize_dt=optimize_dt,
        trajopt_dt=trajopt_dt,
        trajopt_tsteps=trajopt_tsteps,
        trim_steps=trim_steps,
    )
    motion_gen = MotionGen(motion_gen_config)

    print("warming up...")
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False, parallel_finetune=True)

    print("Curobo is Ready")

    add_extensions(simulation_app, args.headless_mode)
```
创建了一个运动规划配置对象 plan_config，设置了参数包括：是否启用图搜索、最大尝试次数、是否启用微调等。  
usd_help.load_stage(my_world.stage): 将场景加载到 USD 布局中。usd_help 可能是一个用于管理 USD 文件操作的辅助类。
usd_help.add_world_to_stage(world_cfg, base_frame="/World"): 将 world_cfg 中定义的世界配置添加到场景中，包括添加障碍物、机器人等。
my_world.scene.add_default_ground_plane(): 向场景中添加默认的地面平面，用于模拟物体与地面的交互。
```
    plan_config = MotionGenPlanConfig(
        enable_graph=False,
        enable_graph_attempt=2,
        max_attempts=max_attempts,
        enable_finetune_trajopt=True,
        parallel_finetune=True,
    )

    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
```
仿真循环
```
    cmd_plan = None    # 存储机器人的运动计划
    cmd_idx = 0    # 追踪当前机器人运动计划的执行进度，初始值为 0，表示当前执行的是运动计划的第一个步骤。
    my_world.scene.add_default_ground_plane()
    i = 0        # 计数器变量 i，记录仿真循环的次数。
    spheres = None    # 运动过程中可能需要可视化的球体信息
    past_cmd = None    # 存储上一步的机器人命令
    while simulation_app.is_running():
        my_world.step(render=True)    # render=True 表示需要进行渲染
        if not my_world.is_playing():
            if i % 100 == 0:
                print("**** Click Play to start simulation *****")
            i += 1
            continue

        step_index = my_world.current_time_step_index
        # print(step_index)
        if step_index < 2:
            my_world.reset()
            robot._articulation_view.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)

            robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )
        if step_index < 20:
            continue

        if step_index == 50 or step_index % 1000 == 0.0:
            print("Updating world, reading w.r.t.", robot_prim_path)
            obstacles = usd_help.get_obstacles_from_stage(
                # only_paths=[obstacles_path],
                reference_prim_path=robot_prim_path,
                ignore_substring=[
                    robot_prim_path,
                    "/World/target",
                    "/World/defaultGroundPlane",
                    "/curobo",
                ],
            ).get_collision_check_world()
            print(len(obstacles.objects))

            motion_gen.update_world(obstacles)
            print("Updated World")
            carb.log_info("Synced CuRobo world from stage.")
```
获取目标物体（通常是立方体）在世界坐标系中的位置 cube_position 和方向 cube_orientation；检查 past_pose 是否为 None。如果是，则将 cube_position 赋值给 past_pose；检查 target_pose 是否为 None。如果是，则将 cube_position 赋值给 target_pose。
```
        cube_position, cube_orientation = target.get_world_pose()

        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
```
获取机器人的关节状态 sim_js，其中包括关节的位置和速度等信息；获取机器人的关节名称列表 sim_js_names。检查关节状态中是否存在任何 NaN（非数值）值.  
cu_js 用来表示机器人当前的关节状态  
args.reactive反应式控制是指机器人能够根据感知到的环境信息或传感器数据进行即时的动作调整，而不是依赖于预先定义的运动规划或控制策略。  
如果处于反应式模式 (args.reactive) 并且之前的命令存在 (past_cmd is not None)，则将 cu_js 的位置、速度和加速度设置为之前命令的相应值。
```
        sim_js = robot.get_joints_state()
        sim_js_names = robot.dof_names
        if np.any(np.isnan(sim_js.positions)):
            log_error("isaac sim has returned NAN joint position values.")
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities),  # * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )

        if not args.reactive:
            cu_js.velocity *= 0.0
            cu_js.acceleration *= 0.0

        if args.reactive and past_cmd is not None:
            cu_js.position[:] = past_cmd.position
            cu_js.velocity[:] = past_cmd.velocity
            cu_js.acceleration[:] = past_cmd.acceleration
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)
```
args.visualize_spheres如果设置为True，则表示需要进行球体可视化。step_index检查当前时间步是否为偶数，减少可视化的频率，以便降低渲染负载。  
sph_list存储了机器人的关节状态的球体表示，是一个列表，其中每个元素代表一个球体。每个球体包含了关于其位置、半径和颜色的信息，用于在三维空间中表示机器人的特定关节状态，将机器人的关节状态转换为球体表示，可以更直观地理解和可视化机器人在空间中的位置和姿态。
```
        if args.visualize_spheres and step_index % 2 == 0:
            sph_list = motion_gen.kinematics.get_robot_as_spheres(cu_js.position)
```
**更新或创建**用于可视化机器人关节状态的**球体对象**。如果spheres为空（即第一次迭代），则创建一个空的spheres列表，并通过迭代sph_list[0]中的每个球体来创建新的球体对象，通过 enumerate() 函数，你可以获取每个球体对象的索引 si 和对应的值 s，即每个球体对象本身。通过调用 append() 方法，你将新创建的 VisualSphere 对象 sp 添加到 spheres 列表的末尾。  
如果spheres不为空，则更新现有的球体对象。对于sph_list[0]中的每个球体，如果其位置不是NaN（表示位置有效），则更新对应索引处的spheres列表中的球体对象的位置和半径。
```
            if spheres is None:
                spheres = []
                # create spheres:

                for si, s in enumerate(sph_list[0]):
                    sp = sphere.VisualSphere(
                        prim_path="/curobo/robot_sphere_" + str(si),
                        position=np.ravel(s.position),
                        radius=float(s.radius),
                        color=np.array([0, 0.8, 0.2]),
                    )
                    spheres.append(sp)
            else:
                for si, s in enumerate(sph_list[0]):
                    if not np.isnan(s.position[0]):
                        spheres[si].set_world_pose(position=np.ravel(s.position))
                        spheres[si].set_radius(float(s.radius))
```
判断是否需要执行末端执行器的运动控制，并设置相应的目标位置和方向
（1） 如果机器人的任何关节的速度（绝对值）小于0.2或采用反应式，则机器人静止；
（2） 如果机器人的末端执行器（EE）的当前位置与目标位置的欧氏距离大于 1e-3，且过去的末端执行器位置与当前末端执行器位置的欧氏距离等于0.0，并且机器人处于静止状态（robot_static 为 True）  
欧氏距离是欧几里得空间中两点之间的直线距离。
```
        robot_static = False
        if (np.max(np.abs(sim_js.velocities)) < 0.2) or args.reactive:
            robot_static = True
        if (
            np.linalg.norm(cube_position - target_pose) > 1e-3
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and robot_static
        ):
            # 设置ee相应的目标位置和方向为cube
            ee_translation_goal = cube_position
            ee_orientation_teleop_goal = cube_orientation
```
ik_goal 定义了末端执行器的目标位置和姿态，其中 ee_translation_goal 表示目标位置，ee_orientation_teleop_goal 表示目标姿态，Pose是curobo内置函数。  
motion_gen.plan_single() 函数用于规划单个运动，它接收当前机器人状态 cu_js 和末端执行器目标姿态 ik_goal 作为输入，并根据这些信息规划出一个运动路径，输出运动规划的结果 result。  
如果运动规划成功（succ 为 True），则可以执行相应的动作；否则，可能需要进行错误处理或者采取其他措施。
```
            ik_goal = Pose(
                position=tensor_args.to_device(ee_translation_goal),
                quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
            )

            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
```
检查规划是否成功，获取插值后的轨迹、完整的关节状态；  
**筛选公共关节**：  遍历所有关节，筛选出同时存在于仿真关节和规划关节中的关节，获取它们的索引和名称。创建空列表 idx_list 和 common_js_names 用于存储公共关节的索引和名称，对于每个关节名称 x，检查它是否也存在于规划关节名称列表 cmd_plan.joint_names 中，如果关节名称 x 在规划关节名称列表中存在，则使用 robot.get_dof_index(x) 获取该关节在机器人关节中的索引，并将索引添加到 idx_list 中。  
将规划好的关节状态重新按照公共关节名称排序，并重置命令索引cmd_idx为0。  
cmd_plan三次赋值：从 result 中获取插值后的运动计划——>将第一步得到的插值后的运动计划转换为完整的关节状态——>根据共同的关节名称对第二步得到的完整关节状态进行排序
如果没有成功规划路径，则记录警告信息。
```
            succ = result.success.item()  # ik_result.success.item()
            if succ:
                cmd_plan = result.get_interpolated_plan()
                cmd_plan = motion_gen.get_full_js(cmd_plan)
                idx_list = []
                common_js_names = []
                for x in sim_js_names:
                    if x in cmd_plan.joint_names:
                        idx_list.append(robot.get_dof_index(x))
                        common_js_names.append(x)

                cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
                cmd_idx = 0

            else:
                carb.log_warn("Plan did not converge to a solution.  No action is being taken.")
            target_pose = cube_position
        past_pose = cube_position
```
如果cmd_plan存在，有运动计划
创建一个 ArticulationAction 对象，包含了当前步骤的关节位置和速度信息，以及关节索引信息；  
articulation_controller.apply_action(art_action) 通过关节动作控制器将动作应用到机器人关节上，以实现运动计划中的目标。更新计数器和状态。  
在不渲染的情况下前进仿真世界两个步骤；检查是否已经达到了运动计划的最后一个命令，如果是，则重置命令索引。
```
        if cmd_plan is not None:       
            cmd_state = cmd_plan[cmd_idx]   #从运动计划中获取当前步骤的关节状态
            past_cmd = cmd_state.clone()    #将当前的关节状态保存为过去的命令
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),   
                cmd_state.velocity.cpu().numpy(),
                joint_indices=idx_list,     
            )
            # set desired joint angles obtained from IK:
            articulation_controller.apply_action(art_action)
            cmd_idx += 1                    #增加命令索引，以便获取下一个命令
            for _ in range(2):
                my_world.step(render=False)
            if cmd_idx >= len(cmd_plan.position):   #检查是否达到了运动计划的最后一个命令
                cmd_idx = 0
                cmd_plan = None
                past_cmd = None
    simulation_app.close()

if __name__ == "__main__":
    main()
```

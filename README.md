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

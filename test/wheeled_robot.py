from isaacsim import SimulationApp

launch_config = {"headless": False, "height": 1080, "width": 1920, "hide_ui": True, "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(launch_config)

import time
def printfps(frame_count, start_time):
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = frame_count / elapsed_time if elapsed_time > 0 else 0
    print(f"FPS: {fps:.2f}", end="\r")    
    frame_count += 1
    return frame_count
1
def optimise():
    import carb
    settings = carb.settings.get_settings()
    simulation_app.update()
    settings.set("/rtx/post/dlss/execMode", 0)





from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.robots import WheeledRobot 
from isaacsim.storage.native import get_assets_root_path

# Scene setup
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
car = world.scene.add(WheeledRobot(
                            prim_path="/World/Car", 
                            name="my_car",
                            wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
                            create_robot=True,
                            usd_path=asset_path,
                    ))




world.reset()
optimise()

# Allow physics to stabilize after reset
for _ in range(10):
    world.step(render=False)

frame_count = 0
start_time= time.time()
print(car.dof_names)

while simulation_app.is_running():
    world.step(render=True)
    frame_count = printfps(frame_count, start_time)
    # car.set_wheel_velocities([2, 2])   gives error
    car.set_linear_velocity([0.2, 0.2, 0])  # inherits this from Robot class
    car.apply_wheel_actions(ArticulationAction(joint_velocities=[2, 2])) # one of the few additional methods in WheeledRobot


# closing the app
simulation_app.close()
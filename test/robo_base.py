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

def optimise():
    import carb
    settings = carb.settings.get_settings()
    simulation_app.update()
    settings.set("/rtx/post/dlss/execMode", 0)





import omni
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.api.robots import Robot
from isaacsim.storage.native import get_assets_root_path

# Scene setup
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Car")
car = world.scene.add(Robot(prim_path="/World/Car", name="my_car"))




world.reset()
optimise()
frame_count = 0
start_time= time.time()
print(car.dof_names)

while simulation_app.is_running():
    world.step(render=True)
    frame_count = printfps(frame_count, start_time)
    # car.apply_action(ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=[0, 3, 3, 0, 0, 0, 0]))
    # car.set_joint_velocities([0, 2, 2, 0, 0, 0, 0])
    # car.set_linear_velocity([0.2, 0.2, 0])
    car.set_world_velocity([0.2, 0.2, 0, 0, 0, 0])


# closing the app
simulation_app.close()
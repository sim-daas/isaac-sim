from isaacsim import SimulationApp

launch_config = {"headless": False, "height": 720, "width": 1280, "hide_ui": False, "renderer": "RayTracedLighting"}
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
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

# Scene setup
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

import os 
home_path = os.path.expanduser("~")
asset_path = os.path.join(home_path, "gitrepos/isaac-sim/test")
robo_path = asset_path + "/drone.usd"
print(robo_path)
add_reference_to_stage(usd_path=robo_path, prim_path="/World/Drone")
 






world.reset()
optimise()
frame_count = 0
start_time= time.time()

while simulation_app.is_running():
    # 1. Step the world
    world.step(render=True)
    frame_count = printfps(frame_count, start_time)











    
    
    
   
# closing the app
simulation_app.close()
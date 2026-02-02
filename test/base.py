from isaacsim import SimulationApp

launch_config = {"headless": False, "height": 720, "width": 1280, "hide_ui": True, "renderer": "RayTracedLighting"}
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
    # settings.set("/rtx/post/dlss/auto", False)
    settings.set("/rtx/post/dlss/execMode", 0)





import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid


# Scene setup
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()









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
import time
from isaacsim import SimulationApp

launch_config = {"headless": False, "height": 720, "width": 1280, "hide_ui": True, "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(launch_config)

import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid

# Scene setup
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()









world.reset()
import carb
settings = carb.settings.get_settings()

# Perform a few steps to let the renderer initialize
for _ in range(10):
    simulation_app.update()

# Slam the settings again
settings.set("/rtx/post/dlss/auto", False)
settings.set("/rtx/post/dlss/execMode", 0)
# settings.set("/rtx/post/dlss/mode", 4) 
# Running the simulation
frame_count = 0
start_time= time.time()

while simulation_app.is_running():
    # 1. Step the world
    world.step(render=True)
    frame_count += 1
    
    # 2. Calculate Elapsed Time
    elapsed_time = time.time() - start_time
    
    # 3. Prevent Division by Zero and Calculate Average
    if elapsed_time > 0:
        avg_fps = frame_count / elapsed_time
        
        # 4. Print to terminal (using \r to keep it on one line)
        print(f"Frames: {frame_count} | Avg FPS: {avg_fps:.2f}", end="\r")











# closing the app
simulation_app.close()
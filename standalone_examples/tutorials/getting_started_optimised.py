import time
import numpy as np
from isaacsim import SimulationApp

config = {
    'headless': False,                    # GUI window (not headless)
    'hide_ui': True,                      # Hide editor UI/menus (key for viewport-only)
    'width': 1920,                        # Viewport width
    'height': 1080,                        # Viewport height
    'window_width': 1920,                 # Window width (slightly larger)
    'window_height': 1080,                 # Window height
}

simulation_app = SimulationApp(config)

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux

# Add Light Source
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# Add Ground Plane
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

# Add Visual Cubes
visual_cube = VisualCuboid(
    prim_path="/visual_cube",
    name="visual_cube",
    position=np.array([0, 0.5, 3.0]),
    size=0.3,
    color=np.array([255, 255, 0]),
)

visual_cube_static = VisualCuboid(
    prim_path="/visual_cube_static",
    name="visual_cube_static",
    position=np.array([0.5, 0, 3.5]),
    size=0.3,
    color=np.array([0, 255, 0]),
)

# Add Physics Cubes
dynamic_cube = DynamicCuboid(
    prim_path="/dynamic_cube",
    name="dynamic_cube",
    position=np.array([0, -0.5, 3.5]),
    size=0.3,
    color=np.array([0, 255, 255]),
)

# start a world to step simulator
my_world = World(stage_units_in_meters=1.0)

# start the simulator
for i in range(3):
    my_world.reset()
    
    import carb
    settings = carb.settings.get_settings()

    # Perform a few steps to let the renderer initialize
    for _ in range(10):
        simulation_app.update()

    # Slam the settings again
    settings.set("/rtx/post/dlss/auto", False)
    settings.set("/rtx/post/dlss/execMode", 0)
    print("simulator running", i)
    if i == 1:
        print("Adding Physics Properties to the Visual Cube")
        from isaacsim.core.prims import RigidPrim

        RigidPrim("/visual_cube")

    if i == 2:
        print("Adding Collision Properties to the Visual Cube")
        from isaacsim.core.prims import GeometryPrim

        prim = GeometryPrim("/visual_cube")
        prim.apply_collision_apis()
        
    frame_count=0
    start_time = time.time()

    for j in range(200):
        my_world.step(render=True)  # stepping through the simulation
        frame_count += 1
        
        # 2. Calculate Elapsed Time
        elapsed_time = time.time() - start_time
        
        # 3. Prevent Division by Zero and Calculate Average
        if elapsed_time > 0:
            avg_fps = frame_count / elapsed_time
            
            # 4. Print to terminal (using \r to keep it on one line)
            print(f"Frames: {frame_count} | Avg FPS: {avg_fps:.2f}", end="\r")

# shutdown the simulator automatically
simulation_app.close()

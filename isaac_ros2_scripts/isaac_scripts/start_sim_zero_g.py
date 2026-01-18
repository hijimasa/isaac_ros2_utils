# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""
Isaac Sim launcher with REST API server.

This script starts Isaac Sim with a REST API server that allows external
applications to control the simulation via HTTP requests.

Usage:
    python start_sim_with_rest_api.py <usd_path> [internal_fps] [physics_steps_per_sec] [real_fps] [headless] [api_port]

Example:
    python start_sim_with_rest_api.py /path/to/stage.usd 60 360 60 false 8080

API Endpoints:
    GET  /health              - Health check
    POST /spawn_robot         - Spawn robot from URDF
    POST /add_usd             - Add USD asset to stage
    POST /simulation/play     - Start simulation
    POST /simulation/pause    - Pause simulation
    POST /simulation/stop     - Stop simulation

API Documentation:
    http://localhost:8080/docs (Swagger UI)
"""

import sys
import time
import signal
from isaacsim import SimulationApp
from distutils.util import strtobool

real_frame_per_second = 0.0
internal_frame_per_second = 60.0
time_steps_per_second = 360
kit = None
is_processing = False
rest_api_server = None


def scheduler(signum, frame):
    global kit
    global is_processing
    global rest_api_server

    if is_processing == False:
        is_processing = True

        # Process REST API commands before updating
        if rest_api_server is not None:
            rest_api_server.process_commands()

        kit.update()
        is_processing = False


def main():
    global kit
    global rest_api_server

    args = sys.argv
    usd_path = args[1]
    if len(args) >= 3:
        internal_frame_per_second = float(args[2])
    else:
        internal_frame_per_second = 60.0
    if len(args) >= 4:
        time_steps_per_second = float(args[3])
    else:
        time_steps_per_second = 360.0
    if len(args) >= 5:
        real_frame_per_second = float(args[4])
    else:
        real_frame_per_second = internal_frame_per_second
    is_headless_mode = False
    if len(args) >= 6:
        is_headless_mode = bool(strtobool(args[5]))
    api_port = 8080
    if len(args) >= 7:
        api_port = int(args[6])

    # Move usd file to /tmp folder
    import shutil
    shutil.copy(usd_path, "/tmp/" + usd_path.split("/")[-1])
    usd_path = "/tmp/" + usd_path.split("/")[-1]

    # URDF import, configuration and simulation sample
    if is_headless_mode:
        kit = SimulationApp({"renderer": "RayTracedLighting", "headless": True, "hide_ui": False, "open_usd": usd_path})
    else:
        kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "open_usd": usd_path})

    import omni
    from isaacsim.core.utils.extensions import enable_extension, disable_extension
    from isaacsim.core.api import SimulationContext, World

    # Enable required extensions
    # Note: omni.isaac.repl is no longer needed as we use REST API
    enable_extension("omni.kit.scripting")
    kit.update()
    enable_extension("omni.graph.bundle.action")
    kit.update()
    enable_extension("omni.graph.visualization.nodes")
    kit.update()
    enable_extension("omni.graph.window.action")
    kit.update()
    enable_extension("omni.graph.window.generic")
    kit.update()
    enable_extension("isaacsim.ros2.bridge")
    if is_headless_mode:
        kit.update()
        enable_extension("omni.isaac.sim.headless.native")

    my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / internal_frame_per_second, rendering_dt=1.0 / internal_frame_per_second)

    import omni.kit.commands
    from pxr import Sdf, Gf, UsdPhysics, PhysxSchema

    omni.usd.get_context().open_stage(usd_path)

    # Get stage handle
    stage_handle = omni.usd.get_context().get_stage()

    # Enable physics
    scene = UsdPhysics.Scene.Define(stage_handle, Sdf.Path("/physicsScene"))
    # Set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    scene.CreateGravityMagnitudeAttr().Set(0.0)
    # Set solver settings
    PhysxSchema.PhysxSceneAPI.Apply(stage_handle.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage_handle, "/physicsScene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("PGS")
    physxSceneAPI.CreateTimeStepsPerSecondAttr(time_steps_per_second)

    # Start REST API server
    import rest_api_server as api_module
    rest_api_server = api_module.create_server(host="0.0.0.0", port=api_port)
    rest_api_server.start()

    # Setup scheduler for real-time simulation
    signal.signal(signal.SIGALRM, scheduler)
    signal.setitimer(signal.ITIMER_REAL, 1/real_frame_per_second, 1/real_frame_per_second)

    print(f"[Isaac Sim] Simulation ready. REST API available at http://0.0.0.0:{api_port}")
    print(f"[Isaac Sim] API documentation: http://localhost:{api_port}/docs")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Shutdown and exit
        print("\n[Isaac Sim] Shutting down...")
        omni.timeline.get_timeline_interface().stop()
        kit.close()


if __name__ == '__main__':
    main()

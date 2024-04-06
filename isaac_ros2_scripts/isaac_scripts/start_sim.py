import sys
import time
import signal
from omni.isaac.kit import SimulationApp

frame_per_second = 60.0
time_steps_per_second = 360
kit = None
is_processing = False

def scheduler(signum, frame):
    global kit
    global is_processing
    if is_processing == False:
        is_processing = True
        kit.update()
        is_processing = False


def main():
    global kit

    args = sys.argv
    usd_path = args[1]
    if len(args) >= 3:
        frame_per_second = float(args[2])
    if len(args) >= 4:
        time_steps_per_second = float(args[3])
    
    # URDF import, configuration and simulation sample
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "open_usd": usd_path})

    import omni
    from omni.isaac.core.utils.extensions import enable_extension, disable_extension
    from omni.isaac.core import SimulationContext, World

    enable_extension("omni.isaac.ros2_bridge")    
    kit.update()
    enable_extension("omni.isaac.repl")
    
    my_world = World(stage_units_in_meters = 1.0, physics_dt = 1.0 / frame_per_second, rendering_dt = 1.0 / frame_per_second)

    import omni.kit.commands
    from pxr import Sdf, Gf, UsdPhysics, PhysxSchema
    
    # Get stage handle
    stage_handle = omni.usd.get_context().get_stage()

    # Enable physics
    scene = UsdPhysics.Scene.Define(stage_handle, Sdf.Path("/physicsScene"))
    # Set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    # Set solver settings
    PhysxSchema.PhysxSceneAPI.Apply(stage_handle.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage_handle, "/physicsScene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    # Refer to https://forums.developer.nvidia.com/t/wheeled-robot-incorrect-behavior/245133
    #physxSceneAPI.CreateSolverTypeAttr("TGS")
    physxSceneAPI.CreateSolverTypeAttr("PGS")
    physxSceneAPI.CreateTimeStepsPerSecondAttr(time_steps_per_second)

    # Start simulation
    #omni.timeline.get_timeline_interface().play()

    signal.signal(signal.SIGALRM, scheduler)
    signal.setitimer(signal.ITIMER_REAL, 1/frame_per_second, 1/frame_per_second)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Shutdown and exit
        omni.timeline.get_timeline_interface().stop()
        kit.close()
    
if __name__ == '__main__':
    main()

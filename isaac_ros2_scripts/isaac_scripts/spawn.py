import math
import xml.etree.ElementTree as ET
import omni
import omni.kit.commands
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.materials import PhysicsMaterial
from omni.importer.urdf import _urdf

def main(urdf_path:str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, fixed=False):
    import search_joint_and_link

    urdf_interface = _urdf.acquire_urdf_interface()
    
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.fix_base = fixed
    import_config.default_drive_strength = 100000.0
    import_config.default_position_drive_damping = 100000.0
    import_config.distance_scale = 1

    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", 
        urdf_path=urdf_path, 
        import_config=import_config,
        get_articulation_root=True,
        )

    status, urdf = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=import_config, )
    kinematics_chain = urdf_interface.get_kinematic_chain(urdf)

    urdf_root = ET.parse(urdf_path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break

    # create a rigid body physical material
    #material = PhysicsMaterial(
    #    prim_path="/World/physics_material/urdf_default_material",  # path to the material prim to create
    #    dynamic_friction=0.4,
    #    static_friction=1.1,
    #    restitution=0.0
    #)
    #prim_path_list = search_joint_and_link.get_link_prim_path_list(kinematics_chain, "/World/" + robot_name + "/")
    #for prim_path in prim_path_list:
    #    prim = GeometryPrim(prim_path, collision=True)
    #    prim.apply_physics_material(material)
    
    stage = omni.usd.get_context().get_stage()

    obj = stage.GetPrimAtPath(stage_path)
    if obj.IsValid():
        obj_xform = UsdGeom.Xformable(obj)
        xform_ops = obj_xform.GetOrderedXformOps()

        obj_xform.ClearXformOpOrder()

        translate_op = obj_xform.AddTranslateOp()
        translate_op.Set((x, y, z))

        rotate_op = obj_xform.AddRotateXYZOp()
        rotate_op.Set((roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi))

    return obj

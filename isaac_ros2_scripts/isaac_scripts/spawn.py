# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import xml.etree.ElementTree as ET
import omni
import omni.kit.commands
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.materials import PhysicsMaterial
from omni.importer.urdf import _urdf
from omni.physx import utils
from omni.physx.scripts import physicsUtils

def main(urdf_path:str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, fixed=False):
    import search_joint_and_link

    urdf_interface = _urdf.acquire_urdf_interface()
    
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.fix_base = fixed
    import_config.default_drive_strength = 0.0
    import_config.default_position_drive_damping = 0.0
    import_config.distance_scale = 1

    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", 
        urdf_path=urdf_path, 
        import_config=import_config,
        get_articulation_root=True,
        )

    status, urdf = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=import_config, )
    kinematics_chain = urdf_interface.get_kinematic_chain(urdf)

    stage_handle = omni.usd.get_context().get_stage()

    urdf_root = ET.parse(urdf_path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break

    stage = omni.usd.get_context().get_stage()

    for child in urdf_root.findall('./material'):
        dynamic_friction = 0.0
        static_friction = 0.0
        restitution = 0.0
        rigid_body_list = child.findall('./isaac_rigid_body')
        if not len(rigid_body_list) == 0:
            if "dynamic_friction" in rigid_body_list[0].attrib:
                dynamic_friction = float(rigid_body_list[0].attrib["dynamic_friction"])
            if "static_friction" in rigid_body_list[0].attrib:
                static_friction = float(rigid_body_list[0].attrib["static_friction"])
            if "restitution" in rigid_body_list[0].attrib:
                restitution = float(rigid_body_list[0].attrib["restitution"])
        utils.addRigidBodyMaterial(stage, "/World/" + robot_name + "/Looks/material_" + child.attrib["name"], density=None, staticFriction=static_friction, dynamicFriction=dynamic_friction, restitution=restitution)
        
    for link in urdf_root.findall("./link"):
        joint_prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/World/" + robot_name), link.attrib["name"])

        collision_list = link.findall('./collision')
        material_list = link.findall('./visual/material')
        if not len(collision_list) == 0 and  not len(material_list) == 0:
            prim = stage_handle.GetPrimAtPath(joint_prim_path + "/collisions")
            physicsUtils.add_physics_material_to_prim(stage_handle, prim, "/World/" + robot_name + "/Looks/material_" + material_list[0].attrib["name"])
    
    obj = stage.GetPrimAtPath(stage_path)
    if obj.IsValid():
        obj_xform = UsdGeom.Xformable(obj)
        xform_ops = obj_xform.GetOrderedXformOps()

        obj_xform.ClearXformOpOrder()

        translate_op = obj_xform.AddTranslateOp()
        translate_op.Set((x, y, z))

        rotate_op = obj_xform.AddRotateXYZOp()
        rotate_op.Set((roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi))

    urdf_joints = []
    joint_type = []
    for child in urdf_root.findall('./joint'):
        if child.attrib["type"] == "continuous":
            urdf_joints.append(child)
            joint_type.append("angular")
        elif child.attrib["type"] == "revolute":
            urdf_joints.append(child)
            joint_type.append("angular")
        elif child.attrib["type"] == "prismatic":
            urdf_joints.append(child)
            joint_type.append("linear")

    joints_prim_paths = []
    for joint in urdf_joints:
        joint_prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/World/" + robot_name), joint.attrib["name"])
        joints_prim_paths.append(joint_prim_path)

    drive = []
    joint_api = []
    for index in range(len(joints_prim_paths)):
        drive.append(UsdPhysics.DriveAPI.Get(stage_handle.GetPrimAtPath(joints_prim_paths[index]), joint_type[index]))
        joint_api.append(PhysxSchema.PhysxJointAPI(stage_handle.GetPrimAtPath(joints_prim_paths[index])))
        api_list = urdf_joints[index].findall('./isaac_drive_api')
        if not len(api_list) == 0:
            if "damping" in api_list[0].attrib:
                drive[index].CreateDampingAttr().Set(float(api_list[0].attrib["damping"]))
            if "stiffness" in api_list[0].attrib:
                drive[index].CreateStiffnessAttr().Set(float(api_list[0].attrib["stiffness"]))
            if "joint_friction" in api_list[0].attrib:
                joint_api[index].CreateJointFrictionAttr().Set(float(api_list[0].attrib["joint_friction"]))

    return obj

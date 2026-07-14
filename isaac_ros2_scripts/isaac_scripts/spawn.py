# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import tempfile
import xml.etree.ElementTree as ET
import omni
import omni.kit.commands
import omni.usd
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema
# Isaac Sim 6: URDF import commands (URDFCreateImportConfig/URDFParseAndImportFile)
# were removed. Use the new URDFImporter API and reference the generated USD.
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
import isaacsim.core.utils.stage as stage_utils
# Isaac Sim 6: omni.physx no longer re-exports utils at top level
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils

DEFAULT_CONVEX_DECOMPOSITON_COLLISION_SHRINK_WARP = True
DEFAULT_CONVEX_DECOMPOSITON_COLLISION_MAX_CONVEX_HULLS = 32

def main(urdf_path:str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, fixed=False):
    import search_joint_and_link

    urdf_root = ET.parse(urdf_path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break

    import_config = URDFImporterConfig(
        urdf_path=urdf_path,
        usd_path=tempfile.mkdtemp(prefix="isaac_ros2_spawn_"),
        merge_fixed_joints=False,
        collision_type="Convex Decomposition",
        fix_base=None,
        override_joint_stiffness=0.0,
        override_joint_damping=0.0,
    )
    robot_usd_path = URDFImporter(import_config).import_urdf()

    stage_path = "/" + robot_name
    stage_utils.add_reference_to_stage(robot_usd_path, stage_path)

    stage_handle = omni.usd.get_context().get_stage()

    # Isaac Sim 6: URDF links without geometry (e.g. a dummy root link) are not
    # imported as rigid bodies. Joints on such links get anchored to the robot
    # root Xform (a non-body) and every rigid body receives its own
    # ArticulationRootAPI, which breaks the robot into disconnected
    # articulations. Restore the classic single-articulation structure.
    child_links = set()
    for joint_elem in urdf_root.findall("./joint"):
        child_elem = joint_elem.find("child")
        if child_elem is not None:
            child_links.add(child_elem.attrib["link"])
    urdf_root_link_name = None
    for link_elem in urdf_root.findall("./link"):
        if link_elem.attrib["name"] not in child_links:
            urdf_root_link_name = link_elem.attrib["name"]
            break

    root_link_path = None
    if urdf_root_link_name is not None:
        root_link_path = search_joint_and_link.find_prim_path_by_name(
            stage_handle.GetPrimAtPath(stage_path), urdf_root_link_name)

    if root_link_path is not None:
        root_link_prim = stage_handle.GetPrimAtPath(root_link_path)
        if not root_link_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(root_link_prim)
            mass_api = UsdPhysics.MassAPI.Apply(root_link_prim)
            mass_api.CreateMassAttr().Set(0.001)
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.0e-6, 1.0e-6, 1.0e-6))

        # Joints anchored to the robot root Xform belong to the root link
        physics_scope = stage_handle.GetPrimAtPath(stage_path + "/Physics")
        if physics_scope.IsValid():
            for joint_prim in Usd.PrimRange(physics_scope):
                if not joint_prim.IsA(UsdPhysics.Joint):
                    continue
                joint_api = UsdPhysics.Joint(joint_prim)
                for rel in (joint_api.GetBody0Rel(), joint_api.GetBody1Rel()):
                    targets = rel.GetTargets()
                    if targets and targets[0].pathString == stage_path:
                        rel.SetTargets([root_link_path])

        # Keep a single articulation root on the root link
        for prim in Usd.PrimRange(stage_handle.GetPrimAtPath(stage_path)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and prim.GetPath().pathString != root_link_path:
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                prim.RemoveAppliedSchema("NewtonArticulationRootAPI")
        if not root_link_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(root_link_prim)

        # fix_base: anchor the root link to the world at the spawn pose
        if fixed:
            world_joint = UsdPhysics.FixedJoint.Define(stage_handle, stage_path + "/Physics/root_joint")
            world_joint.CreateBody1Rel().SetTargets([root_link_path])
            world_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(x, y, z))
            spawn_rot = (Gf.Rotation(Gf.Vec3d.ZAxis(), yaw * 180.0 / math.pi)
                         * Gf.Rotation(Gf.Vec3d.YAxis(), pitch * 180.0 / math.pi)
                         * Gf.Rotation(Gf.Vec3d.XAxis(), roll * 180.0 / math.pi))
            world_joint.CreateLocalRot0Attr().Set(Gf.Quatf(spawn_rot.GetQuat()))

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
        utils.addRigidBodyMaterial(stage, "/" + robot_name + "/Looks/material_" + child.attrib["name"], density=None, staticFriction=static_friction, dynamicFriction=dynamic_friction, restitution=restitution)

    for link in urdf_root.findall("./link"):
        link_prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/" + robot_name), link.attrib["name"])
        if link_prim_path is None:
            continue

        collision_list = link.findall('./collision')
        if not len(collision_list) == 0:
            # Isaac Sim 6: the importer no longer creates a "collisions" child prim.
            # Collision geometries are direct children of the link prim with
            # PhysicsCollisionAPI applied.
            link_prim = stage_handle.GetPrimAtPath(link_prim_path)
            collision_prims = [child_prim for child_prim in link_prim.GetChildren()
                               if child_prim.HasAPI(UsdPhysics.CollisionAPI)]

            material_list = link.findall('./visual/material')
            for prim in collision_prims:
                if not len(material_list) == 0:
                    material_path = "/" + robot_name + "/Looks/material_" + material_list[0].attrib["name"]
                    if stage_handle.GetPrimAtPath(material_path).IsValid():
                        physicsUtils.add_physics_material_to_prim(stage_handle, prim, material_path)

                token_attr = prim.GetAttribute("physics:approximation")
                if token_attr.IsValid():
                    # Tokenの値を取得
                    token_value = token_attr.Get()
                    if token_value == "convexDecomposition":
                        physx_convexdecomp_api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                        physx_convexdecomp_api.GetShrinkWrapAttr().Set(DEFAULT_CONVEX_DECOMPOSITON_COLLISION_SHRINK_WARP)

                        convex_decomposition_list = collision_list[0].findall('./convex_decomposition')
                        if not len(convex_decomposition_list) == 0:
                            physx_convexdecomp_api.GetMaxConvexHullsAttr().Set(int(convex_decomposition_list[0].attrib["max_convex_hulls"]))
                        else:
                            physx_convexdecomp_api.GetMaxConvexHullsAttr().Set(DEFAULT_CONVEX_DECOMPOSITON_COLLISION_MAX_CONVEX_HULLS)
    
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
        joint_prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/" + robot_name), joint.attrib["name"])
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

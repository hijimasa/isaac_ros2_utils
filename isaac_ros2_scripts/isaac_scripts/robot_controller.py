# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import sys
import time
import math
import struct
import argparse
import xml.etree.ElementTree as ET 
import inspect
import omni.kit.commands
import xml.etree.ElementTree as ET

import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from isaacsim.core.utils.prims import get_articulation_root_api_prim_path
import numpy as np
from pxr import Gf, UsdGeom, Usd, UsdPhysics, PhysxSchema
import omni.graph.core as og
from omni.graph.core import GraphPipelineStage
from isaacsim.core.prims import SingleArticulation

import asyncio
import nest_asyncio

nest_asyncio.apply()

def main(urdf_path:str):
    import search_joint_and_link

    # Isaac Sim 6: URDFParseFile / get_kinematic_chain were removed.
    # Prim paths are resolved by searching the stage by name instead.
    from pxr import Sdf, Gf

    stage_handle = omni.usd.get_context().get_stage()

    urdf_root = ET.parse(urdf_path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break
    urdf_joints = []
    for child in urdf_root.findall('.//ros2_control/joint'):
        urdf_joints.append(child)

    urdf_joint_command_interfaces = []
    for child in urdf_root.findall('.//ros2_control/joint/command_interface'):
        urdf_joint_command_interfaces.append(child.attrib["name"])

    urdf_joint_initial_values = []
    for child in urdf_root.findall('.//ros2_control/joint/state_interface'):
        if child.attrib["name"] == "position":
            param_list = child.findall('./param[@name="initial_value"]')
            if not len(param_list) == 0:
                urdf_joint_initial_values.append(float(param_list[0].text))
            else:
                urdf_joint_initial_values.append(0.0)

    joint_commands_topic_name = ""
    joint_states_topic_name = ""
    for child in urdf_root.findall('.//ros2_control/hardware/param'):
        if child.attrib["name"] == "joint_commands_topic":
            joint_commands_topic_name = child.text
        if child.attrib["name"] == "joint_states_topic":
            joint_states_topic_name = child.text

    joint_name = []
    for joint in urdf_joints:
        joint_name.append(joint.attrib["name"])

    joint_type = []
    for joint in urdf_joints:
        for child in urdf_root.findall('./joint'):
            if child.attrib["name"] == joint.attrib["name"]:
                if child.attrib["type"] == "continuous":
                    joint_type.append("angular")
                elif child.attrib["type"] == "revolute":
                    joint_type.append("angular")
                elif child.attrib["type"] == "prismatic":
                    joint_type.append("linear")
                else:
                    joint_type.append(child.attrib["type"])
                break

    joints_prim_paths = []
    for joint in urdf_joints:
        joints_prim_paths.append(search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/" + robot_name), joint.attrib["name"]))

    drive = []
    for index in range(len(joints_prim_paths)):
        drive.append(UsdPhysics.DriveAPI.Get(stage_handle.GetPrimAtPath(joints_prim_paths[index]), joint_type[index]))

    for child in urdf_root.findall('.//isaac/surface_gripper'):
        offset_x = float(child.find("offset_x").text)
        offset_y = float(child.find("offset_y").text)
        offset_z = float(child.find("offset_z").text)
        axis = child.find("axis").text
        grip_threshold = float(child.find("grip_threshold").text)
        force_limit = float(child.find("force_limit").text)
        torque_limit = float(child.find("torque_limit").text)
        bend_angle = float(child.find("bend_angle").text)
        stiffness = float(child.find("stiffness").text)
        damping = float(child.find("damping").text)
        if child.find("retry_close").text == "False" or child.find("retry_close").text == "false" or child.find("retry_close").text == "0":
            retry_close = False
        else:
            retry_close = True

        prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/" + robot_name), child.attrib["name"])
        if prim_path is None:
            print(f"[robot_controller] Warning: link '{child.attrib['name']}' not found, skipping surface gripper")
            continue
        # Isaac Sim 6: SurfaceGripper is defined by the robot schema
        # (IsaacSurfaceGripper prim + D6 attachment point joints on the parent
        # rigid body). The CreateSurfaceGripper command appends "/SurfaceGripper"
        # to the given path by itself.
        from usd.schema.isaac import robot_schema

        result, gripper_prim = omni.kit.commands.execute(
            "CreateSurfaceGripper",
            prim_path=prim_path,
        )
        gripper_prim_path = gripper_prim.GetPath().pathString

        gripper_prim.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(grip_threshold)
        gripper_prim.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(force_limit)
        gripper_prim.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(torque_limit)
        # retryInterval is the time the gripper keeps trying to close before timing out
        gripper_prim.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(60.0 if retry_close else 0.0)

        # Attachment point: a D6 joint anchored on the gripper link's rigid
        # body, locked on all axes. body0 must be a rigid body, so fall back to
        # the nearest rigid-body ancestor when the gripper link has no geometry.
        body_prim = stage_handle.GetPrimAtPath(prim_path)
        while body_prim.IsValid() and not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body_prim = body_prim.GetParent()
        if not body_prim.IsValid():
            print(f"[robot_controller] Warning: no rigid body found for gripper link '{child.attrib['name']}', skipping surface gripper")
            continue
        body_path = body_prim.GetPath().pathString

        gripper_joint_path = prim_path + "/SurfaceGripperJoint"
        gripper_joint = UsdPhysics.Joint.Define(stage_handle, gripper_joint_path)
        robot_schema.ApplyAttachmentPointAPI(gripper_joint.GetPrim())
        if axis in ("1 0 0", "-1 0 0"):
            forward_axis_token = "X"
        elif axis in ("0 1 0", "0 -1 0"):
            forward_axis_token = "Y"
        else:
            forward_axis_token = "Z"
        gripper_joint.GetPrim().GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set(forward_axis_token)
        for limit in ["rotX", "rotY", "rotZ", "transX", "transY", "transZ"]:
            lim_api = UsdPhysics.LimitAPI.Apply(gripper_joint.GetPrim(), limit)
            lim_api.CreateHighAttr().Set(-1)
            lim_api.CreateLowAttr().Set(1)
        gripper_joint.CreateBody0Rel().SetTargets([body_path])

        # Express the URDF offset (gripper link frame) in the body frame
        link_tf = UsdGeom.Xformable(stage_handle.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        body_tf = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        rel_tf = link_tf * body_tf.GetInverse()
        local_pos = rel_tf.Transform(Gf.Vec3d(offset_x, offset_y, offset_z))
        gripper_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local_pos))
        if axis in ("-1 0 0", "0 -1 0"):
            # 180 deg about Z flips +X/-X and +Y/-Y
            axis_quat = Gf.Quatd(0.0, 0.0, 0.0, 1.0)
        elif axis == "0 0 -1":
            # 180 deg about X flips +Z/-Z
            axis_quat = Gf.Quatd(0.0, 1.0, 0.0, 0.0)
        else:
            axis_quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        rel_quat = rel_tf.RemoveScaleShear().ExtractRotationQuat()
        gripper_joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel_quat * axis_quat))

        # The gripper runtime attaches objects through this joint; it must be
        # disabled while the gripper is open, otherwise it pins the robot body
        # to the world.
        gripper_joint.GetPrim().CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        gripper_joint.GetPrim().CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)

        gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([gripper_joint_path])

        # Create ROS2 control graph for the surface gripper
        # Isaac Sim 6: control the gripper through the
        # SurfaceGripperInterface (open_gripper/close_gripper)
        import omni.graph.core as og

        gripper_script = f'''
from isaacsim.robot.surface_gripper import _surface_gripper

_sg = None
_last_state = None

def setup(db: og.Database):
    global _sg
    _sg = _surface_gripper.acquire_surface_gripper_interface()

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    global _sg, _last_state
    if _sg is None:
        return True

    gripper_path = "{gripper_prim_path}"

    close_gripper = db.inputs.close_cmd
    if _last_state != close_gripper:
        _last_state = close_gripper
        try:
            if close_gripper:
                _sg.close_gripper(gripper_path)
            else:
                _sg.open_gripper(gripper_path)
        except Exception:
            pass
    return True
'''

        keys = og.Controller.Keys
        (ros_surface_gripper_graph, (_, _, script_node), _, _) = og.Controller.edit(
            {
                "graph_path": prim_path + "/SurfaceGripper_ROS2_Graph",
                "evaluator_name": "execution",
                "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("SubscribeToggle", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("GripperScript", "omni.graph.scriptnode.ScriptNode"),
                ],
                keys.CONNECT: [
                    ("OnPhysicsStep.outputs:step", "SubscribeToggle.inputs:execIn"),
                    ("OnPhysicsStep.outputs:step", "GripperScript.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("SubscribeToggle.inputs:messageName", "Bool"),
                    ("SubscribeToggle.inputs:messagePackage", "std_msgs"),
                    ("SubscribeToggle.inputs:topicName", prim_path + "/toggle"),
                ],
            },
        )

        # Create custom input for the script node to receive boolean command
        og.Controller.create_attribute(
            script_node,
            "inputs:close_cmd",
            og.Type(og.BaseDataType.BOOL, 1, 0, og.AttributeRole.NONE),
            og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT,
        )
        script_node.get_attribute("inputs:script").set(gripper_script)

        # Connect subscriber data to script input
        og.Controller.connect(
            prim_path + "/SurfaceGripper_ROS2_Graph/SubscribeToggle.outputs:data",
            prim_path + "/SurfaceGripper_ROS2_Graph/GripperScript.inputs:close_cmd"
        )

        og.Controller.evaluate_sync(ros_surface_gripper_graph)
        print(f"[robot_controller] Created SurfaceGripper at {gripper_prim_path}, ROS2 topic: {prim_path}/toggle")

    for child in urdf_root.findall('.//isaac/thruster'):
        prim_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/" + robot_name), child.attrib["name"])
        if prim_path is None:
            print(f"[robot_controller] Warning: link '{child.attrib['name']}' not found, skipping thruster")
            continue
        
        # Isaac Sim 6: apply_force_at_pos now takes (stage_id, encoded body
        # path, force, pos, mode) and must target a rigid body. The force is
        # applied at the thruster link's world pose, along its local Z axis.
        script_string = """
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysicsSchemaTools
from omni.physx import get_physx_interface

stage = None
stage_id = None
prim_path = None
body_path = None

def setup(db: og.Database):
    global stage, stage_id, prim_path, body_path
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    stage_id = ctx.get_stage_id()
    prim_path = db.inputs.target_path
    # The thruster link may have no geometry, in which case it is not a rigid
    # body; apply the force to the nearest rigid-body ancestor instead.
    body_prim = stage.GetPrimAtPath(prim_path)
    while body_prim.IsValid() and not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        body_prim = body_prim.GetParent()
    body_path = body_prim.GetPath().pathString if body_prim.IsValid() else None

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    global stage, stage_id, prim_path, body_path

    if stage is None or prim_path is None or body_path is None:
        return True

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return True

    force = float(db.inputs.force)
    if abs(force) > 0.001:
        physx_interface = get_physx_interface()
        if physx_interface:
            world_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            force_dir = world_tf.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
            pos = world_tf.ExtractTranslation()
            physx_interface.apply_force_at_pos(
                stage_id,
                PhysicsSchemaTools.sdfPathToInt(body_path),
                (force_dir[0] * force, force_dir[1] * force, force_dir[2] * force),
                (pos[0], pos[1], pos[2]),
                "Force",
            )
    return True
        """

        import omni.graph.core as og

        keys = og.Controller.Keys
        (ros_surface_gripper_graph, (_, _, script_node), _, _) = og.Controller.edit(
            {
                "graph_path": prim_path + "/Thruster_Graph",
                "evaluator_name": "execution",
                "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("SubscribeForce", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("ScriptNode", "omni.graph.scriptnode.ScriptNode"),
                ],
                keys.CONNECT: [
                    ("OnPhysicsStep.outputs:step", "SubscribeForce.inputs:execIn"),
                    ("OnPhysicsStep.outputs:step", "ScriptNode.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("SubscribeForce.inputs:messageName", "Float64"),
                    ("SubscribeForce.inputs:messagePackage", "std_msgs"),
                    ("SubscribeForce.inputs:topicName", prim_path + "/force"),
                ],
            },
        )
        og.Controller.create_attribute(
            script_node,
            "inputs:force",
            og.Type(og.BaseDataType.DOUBLE, 1, 0, og.AttributeRole.NONE),
            og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT,
        )
        og.Controller.create_attribute(
            script_node,
            "inputs:target_path",
            og.Type(og.BaseDataType.TOKEN, 1, 0, og.AttributeRole.NONE),
            og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT,
        )
        script_node.get_attribute("inputs:target_path").set(prim_path)
        script_node.get_attribute("inputs:script").set(script_string)
        
        og.Controller.connect(prim_path + "/Thruster_Graph/SubscribeForce.outputs:data", prim_path + "/Thruster_Graph/ScriptNode.inputs:force")

        og.Controller.evaluate_sync(ros_surface_gripper_graph)

    for index in range(len(joints_prim_paths)):
        if urdf_joint_command_interfaces[index] == "position":
            command = urdf_joint_initial_values[index]
            if joint_type[index] == "angular":
                command = command *180 / math.pi
            drive[index].CreateTargetPositionAttr().Set(command)
            if drive[index].GetStiffnessAttr().Get() == 0:
                drive[index].CreateStiffnessAttr().Set(100000000)

        elif urdf_joint_command_interfaces[index] == "velocity":
            command = urdf_joint_initial_values[index]
            if joint_type[index] == "angular":
                command = command *180 / math.pi
            drive[index].CreateTargetVelocityAttr().Set(command)
            if drive[index].GetDampingAttr().Get() == 0:
                drive[index].CreateDampingAttr().Set(15000)

    art_path = get_articulation_root_api_prim_path("/" + robot_name)

    import omni.graph.core as og

    (ros_control_graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/" + robot_name + "/ActionGraph", "evaluator_name": "execution", "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("ArticulationState", "isaacsim.core.nodes.IsaacArticulationState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TimeSplitter", "isaacsim.core.nodes.IsaacTimeSplitter"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishJointState.inputs:execIn"),
                ("OnPhysicsStep.outputs:step", "ArticulationState.inputs:execIn"),
                ("OnPhysicsStep.outputs:step", "SubscribeJointState.inputs:execIn"),
                ("OnPhysicsStep.outputs:step", "ArticulationController.inputs:execIn"),

                ("ReadSimTime.outputs:simulationTime", "TimeSplitter.inputs:time"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Providing path to /panda robot to Articulation Controller node
                # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you may need to uncomment this line
                ("ArticulationController.inputs:targetPrim", art_path),
                ("ArticulationState.inputs:targetPrim", art_path),
                ("PublishJointState.inputs:messageName", "JointState"),
                ("PublishJointState.inputs:messagePackage", "sensor_msgs"),
                ("PublishJointState.inputs:topicName", joint_states_topic_name),
                ("SubscribeJointState.inputs:messageName", "JointState"),
                ("SubscribeJointState.inputs:messagePackage", "sensor_msgs"),
                ("SubscribeJointState.inputs:topicName", joint_commands_topic_name),
            ],
        },
    )
        
    og.Controller.connect("/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:name", "/" + robot_name + "/ActionGraph/ArticulationController.inputs:jointNames")
    og.Controller.connect("/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:position", "/" + robot_name + "/ActionGraph/ArticulationController.inputs:positionCommand")
    og.Controller.connect("/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:velocity", "/" + robot_name + "/ActionGraph/ArticulationController.inputs:velocityCommand")
    og.Controller.connect("/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:effort", "/" + robot_name + "/ActionGraph/ArticulationController.inputs:effortCommand")

    og.Controller.connect("/" + robot_name + "/ActionGraph/TimeSplitter.outputs:seconds", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:header:stamp:sec")
    og.Controller.connect("/" + robot_name + "/ActionGraph/TimeSplitter.outputs:nanoseconds", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:header:stamp:nanosec")
    og.Controller.connect("/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointNames", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:name")
    og.Controller.connect("/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointPositions", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:position")
    og.Controller.connect("/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointVelocities", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:velocity")
    og.Controller.connect("/" + robot_name + "/ActionGraph/ArticulationState.outputs:measuredJointEfforts", "/" + robot_name + "/ActionGraph/PublishJointState.inputs:effort")
        
    og.Controller.evaluate_sync(ros_control_graph)    


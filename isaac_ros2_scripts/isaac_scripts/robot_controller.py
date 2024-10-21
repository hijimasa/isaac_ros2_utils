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
from omni.isaac.kit import SimulationApp
import xml.etree.ElementTree as ET 
import inspect
import omni.kit.commands
from omni.importer.urdf import _urdf
import xml.etree.ElementTree as ET 

import omni
from omni.isaac.core.utils.extensions import enable_extension, disable_extension
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.utils import stage, extensions, nucleus
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.kit.viewport.utility
import omni.replicator.core as rep
from omni.isaac.core.utils.prims import get_articulation_root_api_prim_path
import numpy as np
from pxr import Gf, UsdGeom, Usd
import omni.graph.core as og
from omni.graph.core import GraphPipelineStage
from omni.isaac.core.utils.prims import set_targets

import asyncio
import nest_asyncio

nest_asyncio.apply()

def main(urdf_path:str):
    import search_joint_and_link

    urdf_interface = _urdf.acquire_urdf_interface()

    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Sdf, Gf, UsdPhysics

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.fix_base = False
    import_config.distance_scale = 1

    status, urdf = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=import_config, )
    kinematics_chain = urdf_interface.get_kinematic_chain(urdf)

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
        joints_prim_paths.append(search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/World/" + robot_name), joint.attrib["name"]))

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

        prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])

        xform_prim = UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), prim_path + "/VacuumPoint")
        xform_api = UsdGeom.XformCommonAPI(xform_prim)
        xform_api.SetTranslate(Gf.Vec3d(offset_x, offset_y, offset_z))
        if axis == "1 0 0":
            xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if axis == "-1 0 0":
            xform_api.SetRotate((0, 0, 180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if axis == "0 1 0":
            xform_api.SetRotate((0, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if axis == "0 -1 0":
            xform_api.SetRotate((0, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if axis == "0 0 1":
            xform_api.SetRotate((0, -90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if axis == "0 0 -1":
            xform_api.SetRotate((0, 90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        import omni.graph.core as og

        keys = og.Controller.Keys
        (ros_surface_gripper_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": prim_path + "/SurfaceGripper_Graph",
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SubscribeClose", "omni.isaac.ros2_bridge.ROS2Subscriber"),
                    ("SubscribeOpen", "omni.isaac.ros2_bridge.ROS2Subscriber"),
                    ("SurfaceGripper", "omni.isaac.surface_gripper.SurfaceGripper"),
                    ("SendCloseEvent", "omni.graph.action.SendCustomEvent"),
                    ("SendOpenEvent", "omni.graph.action.SendCustomEvent"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "SubscribeClose.inputs:execIn"),
                    ("OnTick.outputs:tick", "SubscribeOpen.inputs:execIn"),
                    ("OnTick.outputs:tick", "SurfaceGripper.inputs:onStep"),
                    ("SubscribeClose.outputs:execOut", "SendCloseEvent.inputs:execIn"),
                    ("SendCloseEvent.outputs:execOut", "SurfaceGripper.inputs:Close"),
                    ("SubscribeOpen.outputs:execOut", "SendOpenEvent.inputs:execIn"),
                    ("SendOpenEvent.outputs:execOut", "SurfaceGripper.inputs:Open"),
                ],
                keys.SET_VALUES: [
                    ("SubscribeClose.inputs:messageName", "Bool"),
                    ("SubscribeClose.inputs:messagePackage", "std_msgs"),
                    ("SubscribeClose.inputs:topicName", prim_path + "/close"),
                    ("SubscribeOpen.inputs:messageName", "Bool"),
                    ("SubscribeOpen.inputs:messagePackage", "std_msgs"),
                    ("SubscribeOpen.inputs:topicName", prim_path + "/open"),
                    ("SurfaceGripper.inputs:ParentRigidBody", prim_path),
                    ("SurfaceGripper.inputs:GripPosition", prim_path + "/VacuumPoint"),
                    ("SurfaceGripper.inputs:GripThreshold", grip_threshold),
                    ("SurfaceGripper.inputs:ForceLimit", force_limit),
                    ("SurfaceGripper.inputs:BendAngle", bend_angle * 180 / np.pi),
                    ("SurfaceGripper.inputs:Stiffness", stiffness),
                    ("SurfaceGripper.inputs:Damping", damping),
                    ("SurfaceGripper.inputs:RetryClose", retry_close),
                    ("SendCloseEvent.inputs:eventName", "close"),
                    ("SendOpenEvent.inputs:eventName", "open"),
                ],
            },
        )

        og.Controller.evaluate_sync(ros_surface_gripper_graph)

    for child in urdf_root.findall('.//isaac/thruster'):
        prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])
        
        script_string = """
from omni.isaac.dynamic_control import _dynamic_control

dc = None
art = None

def setup(db: og.Database):
    global dc
    global art
    
    dc = _dynamic_control.acquire_dynamic_control_interface()
    art = dc.get_rigid_body(db.inputs.target_path)
    

def cleanup(db: og.Database):
    pass


def compute(db: og.Database):
    global dc
    global art

    dc.apply_body_force(art, [0.0, 0.0, db.inputs.force], [0.0, 0.0, 0.0], False)
    return True
        """

        import omni.graph.core as og

        keys = og.Controller.Keys
        (ros_surface_gripper_graph, (_, _, script_node), _, _) = og.Controller.edit(
            {
                "graph_path": prim_path + "/Thruster_Graph",
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SubscribeForce", "omni.isaac.ros2_bridge.ROS2Subscriber"),
                    ("ScriptNode", "omni.graph.scriptnode.ScriptNode"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "SubscribeForce.inputs:execIn"),
                    ("OnTick.outputs:tick", "ScriptNode.inputs:execIn"),
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

    art_path = get_articulation_root_api_prim_path("/World/" + robot_name)

    import omni.graph.core as og

    (ros_control_graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/World/" + robot_name + "/ActionGraph", "evaluator_name": "execution", "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPhysicsStep", "omni.isaac.core_nodes.OnPhysicsStep"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2Publisher"),
                ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2Subscriber"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("TimeSplitter", "omni.isaac.core_nodes.IsaacTimeSplitter"),
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
        
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:name", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:jointNames")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:position", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:positionCommand")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:velocity", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:velocityCommand")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:effort", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:effortCommand")

    og.Controller.connect("/World/" + robot_name + "/ActionGraph/TimeSplitter.outputs:seconds", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:header:stamp:sec")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/TimeSplitter.outputs:nanoseconds", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:header:stamp:nanosec")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointNames", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:name")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointPositions", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:position")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/ArticulationState.outputs:jointVelocities", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:velocity")
    og.Controller.connect("/World/" + robot_name + "/ActionGraph/ArticulationState.outputs:measuredJointEfforts", "/World/" + robot_name + "/ActionGraph/PublishJointState.inputs:effort")
        
    og.Controller.evaluate_sync(ros_control_graph)    


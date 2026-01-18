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
from isaacsim.asset.importer.urdf import _urdf
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

    urdf_interface = _urdf.acquire_urdf_interface()

    from pxr import Sdf, Gf

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

        prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/" + robot_name + "/base_link/", child.attrib["name"])
        gripper_prim_path = prim_path + "/SurfaceGripper"

        # Isaac Sim 5.1.0: Use CreateSurfaceGripper command to create the gripper prim
        # This creates the gripper with proper USD schema and action graph
        result, gripper_prim = omni.kit.commands.execute(
            "CreateSurfaceGripper",
            prim_path=gripper_prim_path,
        )

        # Set gripper offset position
        gripper_offset_path = gripper_prim_path + "/SurfaceGripperOffset"
        gripper_offset_prim = stage_handle.GetPrimAtPath(gripper_offset_path)
        if gripper_offset_prim.IsValid():
            xform_api = UsdGeom.XformCommonAPI(gripper_offset_prim)
            xform_api.SetTranslate(Gf.Vec3d(offset_x, offset_y, offset_z))
            if axis == "1 0 0":
                xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elif axis == "-1 0 0":
                xform_api.SetRotate((0, 0, 180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elif axis == "0 1 0":
                xform_api.SetRotate((0, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elif axis == "0 -1 0":
                xform_api.SetRotate((0, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elif axis == "0 0 1":
                xform_api.SetRotate((0, -90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elif axis == "0 0 -1":
                xform_api.SetRotate((0, 90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        # Create ROS2 control graph for the surface gripper
        # Isaac Sim 5.1.0: Use ScriptNode to control SurfaceGripper via Python API
        # The _surface_gripper C++ interface uses update() method with close parameter
        import omni.graph.core as og

        gripper_script = f'''
from isaacsim.robot.surface_gripper import _surface_gripper
import omni.usd

_sg = None
_last_state = None
_initialized = False

def setup(db: og.Database):
    global _sg, _initialized
    _sg = _surface_gripper.acquire_surface_gripper_interface()
    _initialized = False

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    global _sg, _last_state, _initialized
    if _sg is None:
        return True
    
    gripper_path = "{gripper_prim_path}"
    
    # Initialize gripper on first compute
    if not _initialized:
        try:
            _sg.create_surface_gripper(gripper_path)
            _initialized = True
        except:
            pass
        return True
    
    close_gripper = db.inputs.close_cmd
    if _last_state != close_gripper:
        _last_state = close_gripper
        try:
            # Use update method with close=True/False
            _sg.update(gripper_path, close_gripper)
        except Exception as e:
            # Fallback: try toggle method if available
            try:
                if close_gripper:
                    _sg.close_gripper(gripper_path)
                else:
                    _sg.open_gripper(gripper_path)
            except:
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
        prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/" + robot_name + "/base_link/", child.attrib["name"])
        
        # Isaac Sim 5.1.0: Use PhysX rigid body API for applying forces
        # Note: PhysxForceAPI.Apply() takes only one argument (prim)
        script_string = """
import omni.physx
from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from omni.physx import get_physx_interface

stage = None
prim_path = None

def setup(db: og.Database):
    global stage, prim_path
    stage = omni.usd.get_context().get_stage()
    prim_path = db.inputs.target_path

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    global stage, prim_path
    
    if stage is None or prim_path is None:
        return True
    
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return True
    
    # Apply force using PhysX simulation interface
    force = float(db.inputs.force)
    if abs(force) > 0.001:
        # Use PhysX interface to apply force at runtime
        physx_interface = get_physx_interface()
        if physx_interface:
            # Apply force in local Z direction
            physx_interface.apply_force_at_pos(
                prim_path,
                Gf.Vec3f(0.0, 0.0, force),
                Gf.Vec3f(0.0, 0.0, 0.0),
                "Force"
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


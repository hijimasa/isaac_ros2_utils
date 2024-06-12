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
#from omni.isaac.sensor import LidarRtx, Camera
#import omni.isaac.core.utils.numpy.rotations as rot_utils
#import numpy as np
from pxr import Gf, UsdGeom, Usd
import omni.graph.core as og
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

    async def control_loop():
        art = None
        art_path = ""
        while art == None or art == _dynamic_control.INVALID_HANDLE:
            await omni.kit.app.get_app().next_update_async()
            ret = search_joint_and_link.find_articulation_root(stage_handle.GetPrimAtPath("/World/" + robot_name))
            if not ret == None:
                (art, art_path) = ret

        import omni.graph.core as og
    
        (ros_control_graph, _, _, _) = og.Controller.edit(
            {"graph_path": "/World/" + robot_name + "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2Subscriber"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
    
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Providing path to /panda robot to Articulation Controller node
                    # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                    # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you may need to uncomment this line
                    ("ArticulationController.inputs:robotPath", art_path),
                    ("PublishJointState.inputs:targetPrim", art_path),
                    ("PublishJointState.inputs:topicName", "joint_states"),
                    ("SubscribeJointState.inputs:messageName", "JointState"),
                    ("SubscribeJointState.inputs:messagePackage", "sensor_msgs"),
                    ("SubscribeJointState.inputs:topicName", "joint_command"),
                ],
            },
        )
        
        og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:name", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:jointNames")
        og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:position", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:positionCommand")
        og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:velocity", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:velocityCommand")
        og.Controller.connect("/World/" + robot_name + "/ActionGraph/SubscribeJointState.outputs:effort", "/World/" + robot_name + "/ActionGraph/ArticulationController.inputs:effortCommand")

        og.Controller.evaluate_sync(ros_control_graph)

    def loop_in_thread(loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(control_loop())

    loop = asyncio.get_event_loop()
    import threading
    t = threading.Thread(target=loop_in_thread, args=(loop,))
    t.start()
    


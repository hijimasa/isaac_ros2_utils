# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys
import math
import asyncio
import xml.etree.ElementTree as ET 
import omni.kit.commands
from omni.importer.urdf import _urdf

import omni
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.kit.viewport.utility
import omni.replicator.core as rep
from pxr import UsdGeom
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.sensor import ContactSensor
import numpy as np

import nest_asyncio

nest_asyncio.apply()

def main(urdf_path:str):
    import search_joint_and_link

    urdf_interface = _urdf.acquire_urdf_interface()

    from pxr import Gf

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

    viewportId = 1
    for child in urdf_root.findall('.//isaac/sensor'):
        from omni.kit.viewport.utility import get_viewport_from_window_name

        if child.attrib["type"] == "lidar":
            prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])

            _, my_lidar = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/Lidar",
                parent=prim_path,
                config=child.find("config").text,
                translation=(0.0, 0, 0.0),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            )

            hydra_texture = rep.create.render_product(my_lidar.GetPath(), [1, 1], name=child.attrib["name"])

            if int(child.find("sensor_dimension_num").text) == 2:
                writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
                writer.initialize(topicName=prim_path + "/" + child.find("topic").text, frameId=child.attrib["name"])
                writer.attach([hydra_texture])
            else:
                writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
                writer.initialize(topicName=prim_path + "/" + child.find("topic").text, frameId=child.attrib["name"])
                writer.attach([hydra_texture])

        if child.attrib["type"] == "camera":
            image_height = int(child.find("image/height").text)
            image_width = int(child.find("image/width").text)
            aspect_ratio = float(image_width) / float(image_height)
            horizontal_fov_rad = float(child.find("horizontal_fov_rad").text)
            horizontal_focal_length = float(child.find("horizontal_focal_length").text)
            vertical_focal_length = float(child.find("vertical_focal_length").text)
            focus_distance = float(child.find("focus_distance").text)

            horizontal_aperture = math.tan(horizontal_fov_rad / 2.0) * 2.0 * horizontal_focal_length
            vertical_aperture = math.tan(horizontal_fov_rad / aspect_ratio / 2.0) * 2.0 * vertical_focal_length

            prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])

            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(prim_path + "/Camera", "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
            xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetProjectionAttr().Set(child.find("projection").text)
            camera_prim.GetFocalLengthAttr().Set(horizontal_focal_length)
            camera_prim.GetFocusDistanceAttr().Set(focus_distance)
            camera_prim.GetClippingRangeAttr().Set((float(child.find("clip/near").text), float(child.find("clip/far").text)))

            keys = og.Controller.Keys
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Camera_Graph",
                    "evaluator_name": "execution",
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("createRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createRenderProduct.inputs:cameraPrim", prim_path + "/Camera"),
                        ("createRenderProduct.inputs:enabled", True),
                        ("createRenderProduct.inputs:height", image_height),
                        ("createRenderProduct.inputs:width", image_width),
                        ("cameraHelperRgb.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperRgb.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("cameraHelperInfo.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperInfo.inputs:topicName", prim_path + "/camera_info"),
                        ("cameraHelperInfo.inputs:type", "camera_info"),
                    ],
                },
            )

            og.Controller.evaluate_sync(ros_camera_graph)

        if child.attrib["type"] == "depth_camera":
            image_height = int(child.find("image/height").text)
            image_width = int(child.find("image/width").text)
            aspect_ratio = float(image_width) / float(image_height)
            horizontal_fov_rad = float(child.find("horizontal_fov_rad").text)
            horizontal_focal_length = float(child.find("horizontal_focal_length").text)
            vertical_focal_length = float(child.find("vertical_focal_length").text)
            focus_distance = float(child.find("focus_distance").text)

            horizontal_aperture = math.tan(horizontal_fov_rad / 2.0) * 2.0 * horizontal_focal_length
            vertical_aperture = math.tan(horizontal_fov_rad / aspect_ratio / 2.0) * 2.0 * vertical_focal_length

            prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])

            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(prim_path + "/DepthCamera", "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
            xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetProjectionAttr().Set(child.find("projection").text)
            camera_prim.GetFocalLengthAttr().Set(horizontal_focal_length)
            camera_prim.GetFocusDistanceAttr().Set(focus_distance)
            camera_prim.GetClippingRangeAttr().Set((float(child.find("clip/near").text), float(child.find("clip/far").text)))


            keys = og.Controller.Keys
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Depth_Camera_Graph",
                    "evaluator_name": "execution",
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("createRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createRenderProduct.inputs:cameraPrim", prim_path + "/DepthCamera"),
                        ("createRenderProduct.inputs:enabled", True),
                        ("createRenderProduct.inputs:height", image_height),
                        ("createRenderProduct.inputs:width", image_width),
                        ("cameraHelperDepth.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperDepth.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperDepth.inputs:type", "depth"),
                        ("cameraHelperInfo.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperInfo.inputs:topicName", prim_path + "/camera_info"),
                        ("cameraHelperInfo.inputs:type", "camera_info"),
                    ],
                },
            )

            og.Controller.evaluate_sync(ros_camera_graph)

        if child.attrib["type"] == "contact":
            prim_path = search_joint_and_link.search_link_prim_path(kinematics_chain, "/World/" + robot_name + "/", child.attrib["name"])

            async def launch_contact_sensor(prim_path, topic_name):
                sensor = ContactSensor(
                    prim_path = prim_path + "/Contact_Sensor",
                    name = "Contact_Sensor",
                    frequency = 15,
                    translation = np.array([0, 0, 0]),
                    min_threshold = 0.0001,
                    max_threshold = 10000000,
                    radius = -1
                )
                
                await omni.kit.app.get_app().next_update_async()
                
                keys = og.Controller.Keys
                (ros_contact_graph, _, _, _) = og.Controller.edit(
                    {
                        "graph_path": prim_path + "/Contact_Graph",
                        "evaluator_name": "execution",
                    },
                    {
                        keys.CREATE_NODES: [
                            ("OnTick", "omni.graph.action.OnPlaybackTick"),
                            ("readContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
                            ("selectIf", "omni.graph.nodes.SelectIf"),
                            ("falseValue", "omni.graph.nodes.ConstantUChar"),
                            ("trueValue", "omni.graph.nodes.ConstantUChar"),
                            ("makeArray", "omni.graph.nodes.ConstructArray"),
                            ("publishSensorValue", "omni.isaac.ros2_bridge.ROS2PublishImage"),
                        ],
                        keys.CONNECT: [
                            ("OnTick.outputs:tick", "readContactSensor.inputs:execIn"),
                            ("readContactSensor.outputs:execOut", "publishSensorValue.inputs:execIn"),
                            ("readContactSensor.outputs:inContact", "selectIf.inputs:condition"),
                            ("falseValue.inputs:value", "selectIf.inputs:ifFalse"),
                            ("trueValue.inputs:value", "selectIf.inputs:ifTrue"),
                            ("selectIf.outputs:result", "makeArray.inputs:input0"),
                            ("makeArray.outputs:array", "publishSensorValue.inputs:data"),
                        ],
                        keys.SET_VALUES: [
                            ("readContactSensor.inputs:csPrim", prim_path + "/Contact_Sensor"),
                            ("falseValue.inputs:value", 0),
                            ("trueValue.inputs:value", 255),
                            ("makeArray.inputs:arraySize", 3),
                            ("publishSensorValue.inputs:height", 1),
                            ("publishSensorValue.inputs:width", 1),
                            ("publishSensorValue.inputs:topicName", prim_path + "/" + topic_name),
                        ],
                    },
                )

                og.Controller.evaluate_sync(ros_contact_graph)

            def loop_in_thread(loop):
                asyncio.set_event_loop(loop)
                loop.run_until_complete(launch_contact_sensor(prim_path, child.find("topic").text))

            loop = asyncio.get_event_loop()
            import threading
            contact_sensor_thread = threading.Thread(target=loop_in_thread, args=(loop,))
            contact_sensor_thread.start()


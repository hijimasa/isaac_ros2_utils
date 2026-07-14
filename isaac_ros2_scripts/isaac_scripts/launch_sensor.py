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

import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.graph.core as og
from omni.graph.core import GraphPipelineStage
# Isaac Sim 6: isaacsim.sensors.physics was moved to extsDeprecated and is not
# enabled by default anymore, so enable it before importing ContactSensor.
# isaacsim.sensors.physics.nodes (enabled by default) caches
# "isaacsim.sensors.physics" as an empty namespace package, which shadows the
# real package until the cached entry is dropped.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.physics")
import importlib
import sys as _sys
_cached = _sys.modules.get("isaacsim.sensors.physics")
if _cached is not None and getattr(_cached, "__file__", None) is None:
    del _sys.modules["isaacsim.sensors.physics"]
    importlib.invalidate_caches()
from isaacsim.sensors.physics import ContactSensor
import numpy as np

import nest_asyncio

nest_asyncio.apply()


def _resolve_lidar_config(requested: str, sensor_dimension_num: int) -> str:
    """Resolve a URDF lidar config name to one supported by Isaac Sim 6.

    Isaac Sim 6 removed JSON lidar profiles (and the Hokuyo configs); RTX lidars
    are now USD sensor assets resolved by name from SUPPORTED_LIDAR_CONFIGS.
    Unsupported names fall back to a generic rotary lidar.
    """
    try:
        from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS
    except ImportError:
        return requested

    from pathlib import PurePosixPath
    valid_names = set()
    for config_path in SUPPORTED_LIDAR_CONFIGS:
        parts = PurePosixPath(config_path)
        vendor_name = parts.parts[3]
        config_name = parts.stem
        valid_names.add(config_name)
        valid_names.add(config_name.replace("_", " "))
        if config_name.startswith(vendor_name):
            without_vendor = config_name[len(vendor_name) + 1:]
            valid_names.add(without_vendor)
            valid_names.add(without_vendor.replace("_", " "))

    if requested in valid_names:
        return requested
    short_name = requested.split("/")[-1]
    if short_name in valid_names:
        return short_name

    fallback = "Example_Rotary_2D" if sensor_dimension_num == 2 else "Example_Rotary"
    print(f"[launch_sensor] LiDAR config '{requested}' is not supported by this Isaac Sim version "
          f"(JSON lidar profiles were removed in Isaac Sim 6), falling back to '{fallback}'. "
          f"Supported configs: {sorted(valid_names)}")
    return fallback


def main(urdf_path:str):
    import search_joint_and_link

    stage_handle = omni.usd.get_context().get_stage()

    urdf_root = ET.parse(urdf_path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break

    def create_empty_link_xform(stage, robot_prim, robot_name, link_name, urdf_root):
        """Create an Xform for an empty link that wasn't created during URDF import.
        This happens when a link has no visual or collision geometry."""
        # Find the joint that defines this link's parent and transform
        for joint in urdf_root.findall('.//joint'):
            child_elem = joint.find('child')
            if child_elem is not None and child_elem.attrib.get('link') == link_name:
                parent_elem = joint.find('parent')
                origin_elem = joint.find('origin')
                
                if parent_elem is not None:
                    parent_link_name = parent_elem.attrib.get('link')
                    # Find parent prim
                    parent_path = search_joint_and_link.find_prim_path_by_name(robot_prim, parent_link_name)
                    
                    if parent_path is None:
                        print(f"[launch_sensor] Warning: Parent link '{parent_link_name}' not found for '{link_name}'")
                        return None
                    
                    # Create Xform for the empty link under parent
                    new_link_path = parent_path + "/" + link_name
                    xform = UsdGeom.Xform.Define(stage, new_link_path)
                    
                    # Apply transform from joint origin
                    if origin_elem is not None:
                        xyz = origin_elem.attrib.get('xyz', '0 0 0').split()
                        rpy = origin_elem.attrib.get('rpy', '0 0 0').split()
                        
                        translate = Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))
                        
                        xform_api = UsdGeom.XformCommonAPI(xform)
                        xform_api.SetTranslate(translate)
                        
                        # Convert RPY to degrees for rotation
                        roll = float(rpy[0]) * 180.0 / math.pi
                        pitch = float(rpy[1]) * 180.0 / math.pi
                        yaw = float(rpy[2]) * 180.0 / math.pi
                        xform_api.SetRotate((roll, pitch, yaw), UsdGeom.XformCommonAPI.RotationOrderXYZ)
                    
                    print(f"[launch_sensor] Created Xform for empty link: {new_link_path}")
                    return new_link_path
        
        return None

    for child in urdf_root.findall('.//isaac/sensor'):
        if child.attrib["type"] == "lidar":
            # Isaac Sim 5.1.0: Use find_prim_path_by_name for reliable prim lookup
            # search_link_prim_path uses kinematics_chain which may not match actual stage structure
            link_name = child.attrib["name"]
            robot_prim = stage_handle.GetPrimAtPath("/" + robot_name)
            
            if not robot_prim.IsValid():
                print(f"[launch_sensor] Warning: Robot prim /{robot_name} not found, skipping LiDAR sensor")
                continue
            
            prim_path = search_joint_and_link.find_prim_path_by_name(robot_prim, link_name)
            
            if prim_path is None:
                # Empty link without visual/collision - create Xform for it
                print(f"[launch_sensor] Link '{link_name}' not found in stage (may be empty link), creating Xform...")
                prim_path = create_empty_link_xform(stage_handle, robot_prim, robot_name, link_name, urdf_root)
                if prim_path is None:
                    print(f"[launch_sensor] Failed to create Xform for '{link_name}', skipping LiDAR sensor")
                    continue

            # Isaac Sim 5.1.0: Validate parent prim exists before creating sensor
            parent_prim = stage_handle.GetPrimAtPath(prim_path)
            if not parent_prim.IsValid():
                print(f"[launch_sensor] Warning: Parent prim {prim_path} not found, skipping LiDAR sensor")
                continue

            # Isaac Sim 6: lidars are USD sensor assets resolved by name; an
            # unknown config only logs a warning and yields a sensor without a
            # profile (no data), so resolve/fall back before creating.
            sensor_dimension_num = int(child.find("sensor_dimension_num").text)
            lidar_config = _resolve_lidar_config(child.find("config").text, sensor_dimension_num)
            my_lidar = None

            try:
                _, my_lidar = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar",
                    path="/Lidar",
                    parent=prim_path,
                    config=lidar_config,
                    translation=(0.0, 0, 0.0),
                    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
                )
            except Exception as e:
                print(f"[launch_sensor] Failed to create LiDAR: {e}")
                continue

            if my_lidar is None:
                print(f"[launch_sensor] Failed to create LiDAR at {prim_path}, skipping...")
                continue

            # Isaac Sim 5.1.0: Use ROS2RtxLidarHelper node via OmniGraph
            # Create render product for the lidar
            lidar_path = my_lidar.GetPath().pathString
            render_product = rep.create.render_product(lidar_path, [1, 1], name=child.attrib["name"])
            render_product_path = render_product.path
            
            # Determine lidar type based on sensor dimension
            lidar_type = "laser_scan" if sensor_dimension_num == 2 else "point_cloud"
            
            keys = og.Controller.Keys
            (ros_lidar_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Lidar_ROS2_Graph",
                    "evaluator_name": "execution",
                    "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                        ("RtxLidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnPhysicsStep.outputs:step", "RtxLidarHelper.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        ("RtxLidarHelper.inputs:renderProductPath", render_product_path),
                        ("RtxLidarHelper.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("RtxLidarHelper.inputs:frameId", child.attrib["name"]),
                        ("RtxLidarHelper.inputs:type", lidar_type),
                    ],
                },
            )
            og.Controller.evaluate_sync(ros_lidar_graph)
            print(f"[launch_sensor] Created LiDAR at {lidar_path}, type: {lidar_type}, topic: {prim_path}/{child.find('topic').text}")

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

            # Isaac Sim 5.1.0: Use find_prim_path_by_name for reliable prim lookup
            link_name = child.attrib["name"]
            robot_prim = stage_handle.GetPrimAtPath("/" + robot_name)
            prim_path = search_joint_and_link.find_prim_path_by_name(robot_prim, link_name)
            
            if prim_path is None:
                # Empty link without visual/collision - create Xform for it
                print(f"[launch_sensor] Camera link '{link_name}' not found in stage (may be empty link), creating Xform...")
                prim_path = create_empty_link_xform(stage_handle, robot_prim, robot_name, link_name, urdf_root)
                if prim_path is None:
                    print(f"[launch_sensor] Failed to create Xform for '{link_name}', skipping camera sensor")
                    continue

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
                        ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createRenderProduct.inputs:cameraPrim", prim_path + "/Camera"),
                        ("createRenderProduct.inputs:enabled", True),
                        ("createRenderProduct.inputs:height", image_height),
                        ("createRenderProduct.inputs:width", image_width),
                        ("cameraHelperRgb.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperRgb.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperRgb.inputs:type", "rgb"),
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

            # Isaac Sim 5.1.0: Use find_prim_path_by_name for reliable prim lookup
            link_name = child.attrib["name"]
            robot_prim = stage_handle.GetPrimAtPath("/" + robot_name)
            prim_path = search_joint_and_link.find_prim_path_by_name(robot_prim, link_name)
            
            if prim_path is None:
                # Empty link without visual/collision - create Xform for it
                print(f"[launch_sensor] Depth camera link '{link_name}' not found in stage (may be empty link), creating Xform...")
                prim_path = create_empty_link_xform(stage_handle, robot_prim, robot_name, link_name, urdf_root)
                if prim_path is None:
                    print(f"[launch_sensor] Failed to create Xform for '{link_name}', skipping depth camera sensor")
                    continue

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
                        ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createRenderProduct.inputs:cameraPrim", prim_path + "/DepthCamera"),
                        ("createRenderProduct.inputs:enabled", True),
                        ("createRenderProduct.inputs:height", image_height),
                        ("createRenderProduct.inputs:width", image_width),
                        ("cameraHelperDepth.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperDepth.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperDepth.inputs:type", "depth"),
                    ],
                },
            )

            og.Controller.evaluate_sync(ros_camera_graph)

        if child.attrib["type"] == "contact":
            # Isaac Sim 5.1.0: Use find_prim_path_by_name for reliable prim lookup
            link_name = child.attrib["name"]
            robot_prim = stage_handle.GetPrimAtPath("/" + robot_name)
            prim_path = search_joint_and_link.find_prim_path_by_name(robot_prim, link_name)
            
            if prim_path is None:
                # Contact sensors need physics geometry, so can't be on empty links
                print(f"[launch_sensor] Warning: Contact sensor link '{link_name}' not found, skipping contact sensor")
                continue
            
            # Validate prim exists and has collision geometry for contact sensing
            contact_prim = stage_handle.GetPrimAtPath(prim_path)
            if not contact_prim.IsValid():
                print(f"[launch_sensor] Warning: Contact sensor prim at '{prim_path}' is invalid, skipping")
                continue

            contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(stage_handle.GetPrimAtPath(prim_path))
            contact_report_api.CreateThresholdAttr(0.0)

            # Isaac Sim 6: the URDF importer applies the collision API to the
            # collision geometry children of the link, not to the link prim
            # itself. ContactSensor must be parented under a collision-enabled prim.
            sensor_parent_path = prim_path
            if not contact_prim.HasAPI(UsdPhysics.CollisionAPI):
                for child_prim in contact_prim.GetChildren():
                    if child_prim.HasAPI(UsdPhysics.CollisionAPI):
                        sensor_parent_path = child_prim.GetPath().pathString
                        break

            sensor = ContactSensor(
                prim_path = sensor_parent_path + "/Contact_Sensor",
                name = "Contact_Sensor",
                frequency = 60,
                translation = np.array([0, 0, 0]),
                min_threshold = 0.0,
                max_threshold = 10000000,
                radius = -1
            )
                
            keys = og.Controller.Keys
            (ros_contact_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Contact_Graph",
                    "evaluator_name": "execution",
                    "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                        ("readContactSensor", "isaacsim.sensors.physics.IsaacReadContactSensor"),
                        ("publishSensorValue", "isaacsim.ros2.bridge.ROS2Publisher"),
                    ],
                    keys.CONNECT: [
                        ("OnPhysicsStep.outputs:step", "readContactSensor.inputs:execIn"),
                        ("readContactSensor.outputs:execOut", "publishSensorValue.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        ("readContactSensor.inputs:csPrim", sensor_parent_path + "/Contact_Sensor"),
                        ("publishSensorValue.inputs:messageName", "Bool"),
                        ("publishSensorValue.inputs:messagePackage", "std_msgs"),
                        ("publishSensorValue.inputs:topicName", prim_path + "/" + child.find("topic").text),
                    ],
                },
            )
            og.Controller.connect(prim_path + "/Contact_Graph/readContactSensor.outputs:inContact", prim_path + "/Contact_Graph/publishSensorValue.inputs:data")

            og.Controller.evaluate_sync(ros_contact_graph)


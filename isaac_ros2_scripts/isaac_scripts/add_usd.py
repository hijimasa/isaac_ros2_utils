# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import omni
import omni.kit.commands
import omni.usd
from pxr import UsdGeom
import omni.isaac.core.utils.stage as stage_utils

import os

def main(usd_path:str, usd_name:str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    obj = stage_utils.add_reference_to_stage(usd_path, "/World/" + usd_name)
    if obj.IsValid():
        obj_xform = UsdGeom.Xformable(obj)
        xform_ops = obj_xform.GetOrderedXformOps()

        obj_xform.ClearXformOpOrder()

        translate_op = obj_xform.AddTranslateOp()
        translate_op.Set((x, y, z))

        rotate_op = obj_xform.AddRotateXYZOp()
        rotate_op.Set((roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi))

    return obj

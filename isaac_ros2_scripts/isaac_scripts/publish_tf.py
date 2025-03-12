import omni
import omni.graph.core as og
from omni.graph.core import GraphPipelineStage

from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims

def main(robot_name:str, target_link:str):
    import search_joint_and_link

    stage_handle = omni.usd.get_context().get_stage()

    target_path = search_joint_and_link.find_prim_path_by_name(stage_handle.GetPrimAtPath("/World/" + robot_name), target_link)

    def publish_tf(prim_path):
        if not is_prim_path_valid(prim_path):
            raise ValueError(f"path '{prim_path}' is invalid.")

        try:
            # Generate the frame_id. OmniActionGraph will use the last part of
            # the full camera prim path as the frame name, so we will extract it here
            # and use it for the pointcloud frame_id.
            frame_id=prim_path.split("/")[-1]

            # Generate an action graph associated with camera TF publishing.
            ros_graph_path = "/RobotTFActionGraph_" + frame_id

            # If a camera graph is not found, create a new one.
            if not is_prim_path_valid(ros_graph_path):
                (ros_camera_graph, _, _, _) = og.Controller.edit(
                    {
                        "graph_path": ros_graph_path,
                        "evaluator_name": "execution",
                        "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                    },
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnPhysicsStep", "omni.isaac.core_nodes.OnPhysicsStep"),
                            ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ]
                    }
                )

            # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
            og.Controller.edit(
                ros_graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("PublishTF_"+frame_id, "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (ros_graph_path+"/OnPhysicsStep.outputs:step",
                            "PublishTF_"+frame_id+".inputs:execIn"),
                        (ros_graph_path+"/IsaacClock.outputs:simulationTime",
                            "PublishTF_"+frame_id+".inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Add target prims for the USD pose. All other frames are static.
        set_target_prims(
            primPath=ros_graph_path+"/PublishTF_"+frame_id,
            inputName="inputs:targetPrims",
            targetPrimPaths=[prim_path],
        )
        return

    publish_tf(target_path)

    return

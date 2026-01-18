# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""
REST API Server for Isaac Sim

This module provides a FastAPI-based REST API server that runs alongside
Isaac Sim, allowing external applications to control the simulation via HTTP.
"""

import threading
import queue
from typing import Optional
from dataclasses import dataclass, field
from enum import Enum

# FastAPI imports
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
import uvicorn


class CommandType(Enum):
    SPAWN_ROBOT = "spawn_robot"
    ADD_USD = "add_usd"
    PLAY = "play"
    PAUSE = "pause"
    STOP = "stop"


@dataclass
class Command:
    """Command to be executed in the main simulation thread."""
    cmd_type: CommandType
    params: dict = field(default_factory=dict)
    result_queue: queue.Queue = field(default_factory=queue.Queue)


class SpawnRobotRequest(BaseModel):
    """Request model for spawning a robot from URDF."""
    urdf_path: str = Field(..., description="Path to the URDF file")
    x: float = Field(default=0.0, description="X position")
    y: float = Field(default=0.0, description="Y position")
    z: float = Field(default=0.0, description="Z position")
    roll: float = Field(default=0.0, description="Roll angle in radians")
    pitch: float = Field(default=0.0, description="Pitch angle in radians")
    yaw: float = Field(default=0.0, description="Yaw angle in radians")
    fixed: bool = Field(default=False, description="Fix robot base to world")


class AddUsdRequest(BaseModel):
    """Request model for adding a USD asset."""
    usd_path: str = Field(..., description="Path to the USD file")
    prim_name: str = Field(..., description="Name for the prim in the stage")
    x: float = Field(default=0.0, description="X position")
    y: float = Field(default=0.0, description="Y position")
    z: float = Field(default=0.0, description="Z position")
    roll: float = Field(default=0.0, description="Roll angle in radians")
    pitch: float = Field(default=0.0, description="Pitch angle in radians")
    yaw: float = Field(default=0.0, description="Yaw angle in radians")


class ResponseModel(BaseModel):
    """Standard response model."""
    success: bool
    message: str
    data: Optional[dict] = None


class IsaacSimRestApi:
    """REST API server for Isaac Sim control."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.host = host
        self.port = port
        self.app = FastAPI(
            title="Isaac Sim REST API",
            description="REST API for controlling Isaac Sim simulation",
            version="1.0.0"
        )
        self.command_queue: queue.Queue = queue.Queue()
        self._server_thread: Optional[threading.Thread] = None
        self._setup_routes()

    def _setup_routes(self):
        """Setup FastAPI routes."""

        @self.app.get("/health", response_model=ResponseModel)
        async def health_check():
            """Health check endpoint."""
            return ResponseModel(success=True, message="Isaac Sim REST API is running")

        @self.app.post("/spawn_robot", response_model=ResponseModel)
        async def spawn_robot(request: SpawnRobotRequest):
            """
            Spawn a robot from URDF file.

            This endpoint imports a URDF file, sets up the robot controller
            (joint state publisher/subscriber), and configures sensors defined
            in the URDF.
            """
            cmd = Command(
                cmd_type=CommandType.SPAWN_ROBOT,
                params=request.model_dump()
            )
            self.command_queue.put(cmd)

            try:
                result = cmd.result_queue.get(timeout=30.0)
                if result.get("success"):
                    return ResponseModel(
                        success=True,
                        message=f"Robot spawned successfully from {request.urdf_path}",
                        data=result.get("data")
                    )
                else:
                    raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
            except queue.Empty:
                raise HTTPException(status_code=504, detail="Timeout waiting for spawn operation")

        @self.app.post("/add_usd", response_model=ResponseModel)
        async def add_usd(request: AddUsdRequest):
            """Add a USD asset to the stage."""
            cmd = Command(
                cmd_type=CommandType.ADD_USD,
                params=request.model_dump()
            )
            self.command_queue.put(cmd)

            try:
                result = cmd.result_queue.get(timeout=30.0)
                if result.get("success"):
                    return ResponseModel(
                        success=True,
                        message=f"USD asset added: {request.prim_name}",
                        data=result.get("data")
                    )
                else:
                    raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))
            except queue.Empty:
                raise HTTPException(status_code=504, detail="Timeout waiting for add_usd operation")

        @self.app.post("/simulation/play", response_model=ResponseModel)
        async def simulation_play():
            """Start/resume the simulation."""
            cmd = Command(cmd_type=CommandType.PLAY)
            self.command_queue.put(cmd)

            try:
                result = cmd.result_queue.get(timeout=10.0)
                return ResponseModel(success=True, message="Simulation started")
            except queue.Empty:
                raise HTTPException(status_code=504, detail="Timeout")

        @self.app.post("/simulation/pause", response_model=ResponseModel)
        async def simulation_pause():
            """Pause the simulation."""
            cmd = Command(cmd_type=CommandType.PAUSE)
            self.command_queue.put(cmd)

            try:
                result = cmd.result_queue.get(timeout=10.0)
                return ResponseModel(success=True, message="Simulation paused")
            except queue.Empty:
                raise HTTPException(status_code=504, detail="Timeout")

        @self.app.post("/simulation/stop", response_model=ResponseModel)
        async def simulation_stop():
            """Stop the simulation."""
            cmd = Command(cmd_type=CommandType.STOP)
            self.command_queue.put(cmd)

            try:
                result = cmd.result_queue.get(timeout=10.0)
                return ResponseModel(success=True, message="Simulation stopped")
            except queue.Empty:
                raise HTTPException(status_code=504, detail="Timeout")

    def start(self):
        """Start the REST API server in a background thread."""
        def run_server():
            uvicorn.run(
                self.app,
                host=self.host,
                port=self.port,
                log_level="info"
            )

        self._server_thread = threading.Thread(target=run_server, daemon=True)
        self._server_thread.start()
        print(f"[REST API] Server started at http://{self.host}:{self.port}")
        print(f"[REST API] API documentation available at http://{self.host}:{self.port}/docs")

    def process_commands(self):
        """
        Process pending commands from the queue.

        This method should be called from the main simulation thread
        (e.g., in the update loop or scheduler).
        """
        while not self.command_queue.empty():
            try:
                cmd = self.command_queue.get_nowait()
                result = self._execute_command(cmd)
                cmd.result_queue.put(result)
            except queue.Empty:
                break
            except Exception as e:
                cmd.result_queue.put({"success": False, "error": str(e)})

    def _execute_command(self, cmd: Command) -> dict:
        """Execute a command in the simulation context."""
        try:
            if cmd.cmd_type == CommandType.SPAWN_ROBOT:
                return self._spawn_robot(cmd.params)
            elif cmd.cmd_type == CommandType.ADD_USD:
                return self._add_usd(cmd.params)
            elif cmd.cmd_type == CommandType.PLAY:
                return self._simulation_play()
            elif cmd.cmd_type == CommandType.PAUSE:
                return self._simulation_pause()
            elif cmd.cmd_type == CommandType.STOP:
                return self._simulation_stop()
            else:
                return {"success": False, "error": f"Unknown command: {cmd.cmd_type}"}
        except Exception as e:
            import traceback
            return {"success": False, "error": str(e), "traceback": traceback.format_exc()}

    def _spawn_robot(self, params: dict) -> dict:
        """Spawn a robot from URDF with controller and sensor setup."""
        import traceback

        prim_path = None

        # Step 1: Import URDF
        try:
            print(f"[REST API] Importing URDF: {params['urdf_path']}")
            import spawn
            obj = spawn.main(
                urdf_path=params["urdf_path"],
                x=params["x"],
                y=params["y"],
                z=params["z"],
                roll=params["roll"],
                pitch=params["pitch"],
                yaw=params["yaw"],
                fixed=params["fixed"]
            )
            prim_path = obj.GetPath().pathString if obj else None
            print(f"[REST API] URDF imported successfully: {prim_path}")
        except Exception as e:
            print(f"[REST API] Error importing URDF: {e}")
            traceback.print_exc()
            return {"success": False, "error": f"Failed to import URDF: {e}"}

        # Step 2: Setup robot controller (joint state pub/sub)
        try:
            print(f"[REST API] Setting up robot controller...")
            import robot_controller
            robot_controller.main(urdf_path=params["urdf_path"])
            print(f"[REST API] Robot controller setup complete")
        except Exception as e:
            print(f"[REST API] Error setting up robot controller: {e}")
            traceback.print_exc()
            return {"success": False, "error": f"Failed to setup robot controller: {e}", "data": {"prim_path": prim_path}}

        # Step 3: Setup sensors
        try:
            print(f"[REST API] Setting up sensors...")
            import launch_sensor
            launch_sensor.main(urdf_path=params["urdf_path"])
            print(f"[REST API] Sensor setup complete")
        except Exception as e:
            print(f"[REST API] Error setting up sensors: {e}")
            traceback.print_exc()
            return {"success": False, "error": f"Failed to setup sensors: {e}", "data": {"prim_path": prim_path}}

        return {
            "success": True,
            "data": {"prim_path": prim_path}
        }

    def _add_usd(self, params: dict) -> dict:
        """Add a USD asset to the stage."""
        import add_usd

        obj = add_usd.main(
            usd_path=params["usd_path"],
            usd_name=params["prim_name"],
            x=params["x"],
            y=params["y"],
            z=params["z"],
            roll=params["roll"],
            pitch=params["pitch"],
            yaw=params["yaw"]
        )

        prim_path = obj.GetPath().pathString if obj else None
        return {
            "success": True,
            "data": {"prim_path": prim_path}
        }

    def _simulation_play(self) -> dict:
        """Start the simulation timeline."""
        import omni.timeline
        omni.timeline.get_timeline_interface().play()
        return {"success": True}

    def _simulation_pause(self) -> dict:
        """Pause the simulation timeline."""
        import omni.timeline
        omni.timeline.get_timeline_interface().pause()
        return {"success": True}

    def _simulation_stop(self) -> dict:
        """Stop the simulation timeline."""
        import omni.timeline
        omni.timeline.get_timeline_interface().stop()
        return {"success": True}


# Global instance for access from start_sim.py
_api_server: Optional[IsaacSimRestApi] = None


def create_server(host: str = "0.0.0.0", port: int = 8080) -> IsaacSimRestApi:
    """Create and return the REST API server instance."""
    global _api_server
    _api_server = IsaacSimRestApi(host=host, port=port)
    return _api_server


def get_server() -> Optional[IsaacSimRestApi]:
    """Get the current REST API server instance."""
    return _api_server

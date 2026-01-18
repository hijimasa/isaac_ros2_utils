import json
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node


class SpawnRobot(Node):
    def __init__(self):
        super().__init__('spawn_robot')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '':
            self.get_logger().error('urdf_path parameter is required')
            return

        self.declare_parameter('x', 0.0)
        robot_x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        robot_y = self.get_parameter('y').get_parameter_value().double_value
        self.declare_parameter('z', 0.0)
        robot_z = self.get_parameter('z').get_parameter_value().double_value
        self.declare_parameter('R', 0.0)
        robot_roll = self.get_parameter('R').get_parameter_value().double_value
        self.declare_parameter('P', 0.0)
        robot_pitch = self.get_parameter('P').get_parameter_value().double_value
        self.declare_parameter('Y', 0.0)
        robot_yaw = self.get_parameter('Y').get_parameter_value().double_value
        self.declare_parameter('fixed', False)
        robot_fixed = self.get_parameter('fixed').get_parameter_value().bool_value

        # REST API settings
        self.declare_parameter('api_host', 'localhost')
        api_host = self.get_parameter('api_host').get_parameter_value().string_value
        self.declare_parameter('api_port', 8080)
        api_port = self.get_parameter('api_port').get_parameter_value().integer_value

        api_url = f"http://{api_host}:{api_port}/spawn_robot"

        payload = {
            "urdf_path": urdf_path,
            "x": robot_x,
            "y": robot_y,
            "z": robot_z,
            "roll": robot_roll,
            "pitch": robot_pitch,
            "yaw": robot_yaw,
            "fixed": robot_fixed
        }

        self.get_logger().info(f"Spawning robot from {urdf_path}")
        self.get_logger().info(f"Position: ({robot_x}, {robot_y}, {robot_z})")
        self.get_logger().info(f"Orientation: ({robot_roll}, {robot_pitch}, {robot_yaw})")
        self.get_logger().info(f"Fixed: {robot_fixed}")

        try:
            data = json.dumps(payload).encode('utf-8')
            req = urllib.request.Request(
                api_url,
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            with urllib.request.urlopen(req, timeout=60.0) as response:
                result = json.loads(response.read().decode('utf-8'))

                if result.get("success"):
                    self.get_logger().info(f"Robot spawned successfully: {result.get('message')}")
                    if result.get("data"):
                        self.get_logger().info(f"Prim path: {result['data'].get('prim_path')}")
                else:
                    self.get_logger().error(f"Failed to spawn robot: {result.get('message')}")

        except urllib.error.URLError as e:
            self.get_logger().error(
                f"Could not connect to Isaac Sim REST API at {api_url}. "
                f"Make sure Isaac Sim is running with REST API enabled. Error: {e}"
            )
        except TimeoutError:
            self.get_logger().error("Request timed out while spawning robot")
        except Exception as e:
            self.get_logger().error(f"Error spawning robot: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = SpawnRobot()
    node.get_logger().info("spawn_robot node finished")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

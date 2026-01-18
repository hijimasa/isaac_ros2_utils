import os
import json
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node


class AddUsd(Node):
    def __init__(self):
        super().__init__('add_usd')

        self.declare_parameter('usd_path', '')
        usd_path = self.get_parameter('usd_path').get_parameter_value().string_value
        if usd_path == '':
            self.get_logger().error('usd_path parameter is required')
            return

        self.declare_parameter('usd_name', '')
        usd_name = self.get_parameter('usd_name').get_parameter_value().string_value
        if usd_name == '':
            usd_name = os.path.splitext(os.path.basename(usd_path))[0]

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

        # REST API settings
        self.declare_parameter('api_host', 'localhost')
        api_host = self.get_parameter('api_host').get_parameter_value().string_value
        self.declare_parameter('api_port', 8080)
        api_port = self.get_parameter('api_port').get_parameter_value().integer_value

        api_url = f"http://{api_host}:{api_port}/add_usd"

        payload = {
            "usd_path": usd_path,
            "prim_name": usd_name,
            "x": robot_x,
            "y": robot_y,
            "z": robot_z,
            "roll": robot_roll,
            "pitch": robot_pitch,
            "yaw": robot_yaw
        }

        self.get_logger().info(f"Adding USD from {usd_path}")
        self.get_logger().info(f"Prim name: {usd_name}")
        self.get_logger().info(f"Position: ({robot_x}, {robot_y}, {robot_z})")
        self.get_logger().info(f"Orientation: ({robot_roll}, {robot_pitch}, {robot_yaw})")

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
                    self.get_logger().info(f"USD added successfully: {result.get('message')}")
                    if result.get("data"):
                        self.get_logger().info(f"Prim path: {result['data'].get('prim_path')}")
                else:
                    self.get_logger().error(f"Failed to add USD: {result.get('message')}")

        except urllib.error.URLError as e:
            self.get_logger().error(
                f"Could not connect to Isaac Sim REST API at {api_url}. "
                f"Make sure Isaac Sim is running with REST API enabled. Error: {e}"
            )
        except TimeoutError:
            self.get_logger().error("Request timed out while adding USD")
        except Exception as e:
            self.get_logger().error(f"Error adding USD: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = AddUsd()
    node.get_logger().info("add_usd node finished")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

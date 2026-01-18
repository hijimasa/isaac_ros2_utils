import json
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node


class PublishTf(Node):
    def __init__(self):
        super().__init__('publish_tf')

        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        if robot_name == '':
            self.get_logger().error('robot_name parameter is required')
            return

        self.declare_parameter('target_link', '')
        target_link = self.get_parameter('target_link').get_parameter_value().string_value
        if target_link == '':
            self.get_logger().error('target_link parameter is required')
            return

        # REST API settings
        self.declare_parameter('api_host', 'localhost')
        api_host = self.get_parameter('api_host').get_parameter_value().string_value
        self.declare_parameter('api_port', 8080)
        api_port = self.get_parameter('api_port').get_parameter_value().integer_value

        api_url = f"http://{api_host}:{api_port}/publish_tf"

        payload = {
            "robot_name": robot_name,
            "target_link": target_link
        }

        self.get_logger().info(f"Publishing TF for robot: {robot_name}, link: {target_link}")

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
                    self.get_logger().info(f"TF publisher created successfully: {result.get('message')}")
                else:
                    self.get_logger().error(f"Failed to create TF publisher: {result.get('message')}")

        except urllib.error.URLError as e:
            self.get_logger().error(
                f"Could not connect to Isaac Sim REST API at {api_url}. "
                f"Make sure Isaac Sim is running with REST API enabled. Error: {e}"
            )
        except TimeoutError:
            self.get_logger().error("Request timed out while creating TF publisher")
        except Exception as e:
            self.get_logger().error(f"Error creating TF publisher: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = PublishTf()
    node.get_logger().info("publish_tf node finished")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

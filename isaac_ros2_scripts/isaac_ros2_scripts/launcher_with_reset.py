import os
import signal
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
from os.path import expanduser

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('usd_path', '')
        usd_path = self.get_parameter('usd_path').get_parameter_value().string_value
        if usd_path == '':
            usd_path = os.path.join(
                get_package_share_directory('isaac_ros2_scripts'), 'meshes/USD/default_stage.usd')
        self.declare_parameter('fps', 60.0)
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.declare_parameter('real_fps', 0.0)
        real_fps = self.get_parameter('real_fps').get_parameter_value().double_value
        if real_fps == 0.0:
            real_fps = fps
        self.declare_parameter('time_steps_per_second', 600.0)
        time_steps_per_second = self.get_parameter('time_steps_per_second').get_parameter_value().double_value

        # REST API settings
        self.declare_parameter('api_port', 8080)
        api_port = self.get_parameter('api_port').get_parameter_value().integer_value

        self.declare_parameter('isaac_path', '/isaac-sim')
        isaac_path = self.get_parameter('isaac_path').get_parameter_value().string_value
        if os.path.isfile(os.path.join(expanduser("~"), '.local/share/ov/pkg/isaac_sim-2023.1.1', 'python.sh')):
            isaac_path = os.path.join(expanduser("~"), '.local/share/ov/pkg/isaac_sim-2023.1.1')
            
        self.proc = None
        
        python_script = os.path.join(
                    isaac_path, 'python.sh')
        if not os.path.isfile(python_script):
            self.get_logger().fatal('python.sh not found!!')
            return
        
        start_script = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'start_sim_with_reset.py')
        command = ["bash", python_script, start_script, usd_path, str(fps), str(time_steps_per_second), str(real_fps), "False", str(api_port)]
        print(command)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]=os.path.join(
                get_package_share_directory('isaac_ros2_scripts'), 'config/fastdds.xml')
        self.proc = subprocess.Popen(command, preexec_fn=os.setsid)        
    
    def __del__(self):
        if not self.proc == None:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isaac_ros2_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('isaac_scripts/*.py')),
        (os.path.join('share',package_name), glob('isaac_scripts/*.sh')),
        (os.path.join('share',package_name, 'meshes', 'USD'), glob('meshes/USD/*.usd')),
        (os.path.join('share',package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launcher = isaac_ros2_scripts.launcher:main',
            'launcher_with_reset = isaac_ros2_scripts.launcher_with_reset:main',
            'launcher_zero_g = isaac_ros2_scripts.launcher_zero_g:main',
            'launcher_with_headless = isaac_ros2_scripts.launcher_with_headless:main',
            'spawn_robot = isaac_ros2_scripts.spawn_robot:main',
            'prepare_sensors = isaac_ros2_scripts.prepare_sensors:main',
            'prepare_robot_controller = isaac_ros2_scripts.prepare_robot_controller:main',
            'add_usd = isaac_ros2_scripts.add_usd:main',
            'publish_tf = isaac_ros2_scripts.publish_tf:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'inria_unitree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/bridge_launcher.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clemente Donoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='ROS 2 package for Dynamixel 6-DOF input control',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_bridge = inria_unitree.ros_bridge:main',
            'cartesian_controller = inria_unitree.cartesian_controller:main',
            'joint_controller = inria_unitree.joint_controller:main',
            'interactive_marker = inria_unitree.interactive_marker:main',
            'robot_state = inria_unitree.robot_state:main',
            'loco_client = inria_unitree.loco_client:main',
            'joystick = inria_unitree.joystick:main',
        ],
    },
)

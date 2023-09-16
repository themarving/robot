from setuptools import find_packages, setup

package_name = 'robot_main_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='name',
    maintainer_email='name@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_comms_node = robot_main_pkg.lidar_comms_node:main",
            "motor_control_node = robot_main_pkg.motor_control_node:main",
            "main_control_node = robot_main_pkg.main_control_node:main",
            "ultrasonic_sensors_node = robot_main_pkg.ultrasonic_sensors_node:main",
            "led_control_node = robot_main_pkg.led_control_node:main"
        ],
    },
)

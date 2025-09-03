from setuptools import find_packages, setup

package_name = 'robot_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'dynamixel-sdk'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angle_publisher_2 = robot_control_pkg.angle_publisher_2:main',
            'angle_publisher = robot_control_pkg.angle_publisher:main',
            'dynamixel_ax_controller = robot_control_pkg.dynamixel_ax_controller:main',
            'dynamixel_ax_controller_2 = robot_control_pkg.dynamixel_ax_controller_2:main',
            'main_control = robot_control_pkg.main_control:main',
            'motor_test = robot_control_pkg.motor_test:main',
        ],
    },
)

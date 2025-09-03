from setuptools import find_packages, setup

package_name = 'camtest_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [ 'Yolo_RealSense_Node = camtest_pkg.Yolo_RealSense_Node:main', 'transform_node = camtest_pkg.transform_node:main', 'Scout_node = camtest_pkg.Scout_node:main',
        ],
    },
)

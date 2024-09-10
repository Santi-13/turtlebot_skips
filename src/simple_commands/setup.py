from setuptools import find_packages, setup

package_name = 'simple_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_turtlebot.py']),
        ('share/' + package_name + '/urdf', ['urdf/turtlebot3_burger.urdf.xacro', 'urdf/turtlebot3_burger.gazebo.xacro']),
        ('share/' + package_name + '/meshes', ['meshes/bases/burger_base.stl', 'meshes/sensors/lds.stl', 'meshes/wheels/left_tire.stl', 'meshes/wheels/right_tire.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanmaster',
    maintainer_email='s.penunuri@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_publisher = simple_commands.movement_publisher:main',
            'aruco_publisher = simple_commands.aruco_publisher:main',
            'aruco_subscriber = simple_commands.aruco_subscriber:main'
        ],
    },
)

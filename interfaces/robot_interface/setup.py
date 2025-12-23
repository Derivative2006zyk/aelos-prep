from setuptools import find_packages, setup

package_name = 'robot_interface'

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
    maintainer='derivatives',
    maintainer_email='derivatives@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    pytest_requirements=["pytest"],
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'pub_cmd_vel = robot_interface.pub_cmd_vel:main', #publish/cmd_vel
            'pub_robot_state = robot_interface.pub_robot_state:main', #publish/robot_state
            'sub_heartbeat_b = robot_interface.sub_heartbeat:main',  #subscriber/heartbeat
            'fake_robot_node = robot_interface.fake_robot_node:main',#fake_node to control
        ],
    },
)

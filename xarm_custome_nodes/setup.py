from setuptools import find_packages, setup

package_name = 'xarm_custome_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_nodes.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newtonjeri',
    maintainer_email='newtonkaris45@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_states_subscriber_node = xarm_custome_nodes.joint_states_subscriber_node:main",
            "unity_to_ros2_publisher_node = xarm_custome_nodes.unity_to_ros2_publisher_test_node:main",
            "unity_subscriber_node = xarm_custome_nodes.unity_subscriber:main",
            "tcp_pose_publisher_node = xarm_custome_nodes.tcp_pose_publisher_node:main",
            "data_from_unity_analysis = xarm_custome_nodes.data_from_unity_analysis_node:main",
        ],
    },
)

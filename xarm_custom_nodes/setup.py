from setuptools import find_packages, setup

package_name = 'xarm_custom_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_nodes.launch.py']),
        ('share/' + package_name + '/launch', ['launch/write_data_to_csv.launch.py'])
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
            "joints_info_node = xarm_custom_nodes.joints_info_node:main",
            "joints_info_saver_node = xarm_custom_nodes.joints_info_saver_node:main",
            "unity_to_ros2_publisher_node = xarm_custom_nodes.unity_to_ros2_publisher_test_node:main",
            "unity_subscriber_node = xarm_custom_nodes.unity_subscriber:main",
            "tcp_pose_publisher_node = xarm_custom_nodes.tcp_pose_publisher_node:main",
            "data_from_unity_analysis = xarm_custom_nodes.data_from_unity_analysis_node:main",
            "mode_switcher_node = xarm_custom_nodes.mode_switcher_node:main",
            "time_stamp_subscriber_node = xarm_custom_nodes.time_stamp_subscriber:main"
        ],
    },
)

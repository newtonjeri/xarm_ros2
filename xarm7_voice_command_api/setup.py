from setuptools import find_packages, setup

package_name = 'xarm7_voice_command_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/xarm7_voice_command.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newton',
    maintainer_email='newtonkaris45@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_client_node = xarm7_voice_command_api.alexa_interface:app.run',
        ],
    },
)

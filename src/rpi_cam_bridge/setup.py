from setuptools import setup

package_name = 'rpi_cam_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_cam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@localhost',
    description='Publish Raspberry Pi CSI camera frames to ROS 2 topics via rpicam-vid.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cam_node = rpi_cam_bridge.cam_node:main',
        ],
    },
)

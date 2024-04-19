from setuptools import setup
from os.path import join
from glob import glob

package_name = 'yolov8_isaac_ros'
submodules = "yolov8_isaac_ros/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamlesh',
    maintainer_email='kamlesh.kumar@sjsu.edu',
    description='ROS2 package for YOLOv8 object detection with Nvidia Isaac ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Yolov8Decoder = yolov8_isaac_ros.Yolov8Decoder:main',
            'yolov8_visualizer = yolov8_isaac_ros.yolov8_visualizer:main',
            'image_publisher = yolov8_isaac_ros.image_publisher:main',
            'ip_cam_publisher = yolov8_isaac_ros.ip_cam_neoapi:main',
            'yolov8_engine_visualizer_external = yolov8_isaac_ros.yolov8_engine_visualizer_external:main',
        ],
    },
)

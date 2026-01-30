from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'efficientsam3_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'torch',
        'torchvision',
        'pillow',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 node for dynamic object filtering using EfficientSAM3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_filter_node = efficientsam3_ros2.dynamic_filter_node:main',
        ],
    },
)

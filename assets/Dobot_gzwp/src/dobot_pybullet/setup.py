import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dobot_pybullet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monojitlayek',
    maintainer_email='monojitlayek@todo.todo',
    description='High-Performance PyBullet Physics Simulator and Visualizer for Dobot Nova 5 on macOS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_pybullet_node = dobot_pybullet.dobot_pybullet_node:main',
        ],
    },
)


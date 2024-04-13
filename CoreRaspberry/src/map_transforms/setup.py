from setuptools import setup
from glob import glob
import os

package_name = 'map_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={package_name: 'src/' + package_name},
    py_modules=[
    'map_transforms.odom_to_map',
    'map_transforms.odom_to_map_key',
    'map_transforms.reset_car',
    #'map_transforms.odom_to_map_v2',

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for generating world scenarios in RVIZ',
    license='TODO',
    data_files=[
    #('share/' + package_name + '/worlds', glob(os.path.join('map_transforms', 'worlds', '*.*'))),
    ('share/' + package_name + '/markers', glob(os.path.join('map_transforms', 'markers', '*.*'))),
    ('share/' + package_name, ['package.xml']),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_map = map_transforms.odom_to_map:main',
            'odom_to_map_key = map_transforms.odom_to_map_key:main',
            'reset_car = map_transforms.reset_car:main',
            #'odom_to_map2 = map_transforms.odom_to_map_v2:main',

        ],
    },
)
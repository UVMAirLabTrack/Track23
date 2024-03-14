from setuptools import setup
from glob import glob
import os

package_name = 'world_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={package_name: 'src/' + package_name},
    py_modules=[
    'world_gen.gen',
    'world_gen.four_way_marker',
    'world_gen.parse_and_pass',
    'world_gen.gen_v2',
    'world_gen.open_world_data'
    
    ],
    install_requires=['setuptools','core_functions'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for generating world scenarios in RVIZ',
    license='TODO',
    data_files=[
    (f'share/{package_name}/msg', ['msg/world_marker_msg.msg']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*rviz.*'))),
    ('share/' + package_name + '/worlds', glob(os.path.join('world_gen', 'worlds', '*.*'))),
    ('share/' + package_name + '/markers', glob(os.path.join('world_gen', 'markers', '*.*'))),
    ('share/' + package_name, ['package.xml']),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen = world_gen.gen:main',
            'four_way_marker = world_gen.four_way_marker:main',
            'parse_and_pass = world_gen.parse_and_pass:main',
            'gen_v2 = world_gen.gen_v2:main'
            'core = world_gen.open_world_data:main'
        ],
    },
)
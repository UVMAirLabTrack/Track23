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
    'world_gen.FourWayViz',
    'world_gen.color_publisher',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for generating world scenarios in RVIZ',
    license='TODO',
    data_files=[
    ('share/' + package_name + '/worlds', glob(os.path.join('world_gen', 'worlds', '*.*'))),
    ('share/' + package_name, ['package.xml']),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen = world_gen.gen:main',
            'fourwayviz = world_gen.FourWayViz:main',
            'ct = world_gen.color_publisher:main',
        ],
    },
)
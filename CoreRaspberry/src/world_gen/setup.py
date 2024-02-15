from setuptools import setup, find_packages
import os

package_name = 'world_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for generating world scenarios in RVIZ',
    license='TODO',
    package_data={
        'world_gen': ['worlds/*'],
    },
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen = world_gen.gen:main',
            'fourwayviz = world_gen.FourWayViz:main',
            'ct = world_gen.color_publisher:main',
        ],
    },
)
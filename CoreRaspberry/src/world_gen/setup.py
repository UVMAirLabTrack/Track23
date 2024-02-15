from setuptools import setup
from glob import glob
import os


package_name = 'world_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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

    package_dir={package_name: 'src/' + package_name + '/' + package_name},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('src/' + package_name + '/worlds/*')),
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

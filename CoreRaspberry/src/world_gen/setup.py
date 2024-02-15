from setuptools import setup

package_name = 'world_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen = world_gen.gen:main',
            'fourwayviz = world_gen.FourWayViz:main',
            'ct = world_gen.color_publisher:main',

        ],
    },
)
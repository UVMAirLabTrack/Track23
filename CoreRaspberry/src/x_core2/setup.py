from setuptools import setup

package_name = 'x_core2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'x_core2.parse_and_pass',
        'x_core2.open_world_data',
        'x_core2.pose_strip'



    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for controlling a four-way stoplight using ESP serial communication',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_all_pose = x_core2.parse_and_pass:main',
            'open_world_data = x_core2.open_world_data:main',
            'pose_strip = x_core2.pose_strip:main',
        ],
    },
)
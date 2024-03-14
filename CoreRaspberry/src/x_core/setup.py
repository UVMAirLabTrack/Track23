from setuptools import setup

package_name = 'x_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'x_core.pub',
        'x_core.open_world_data',
        


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
            'pub = x_core.pub:main',
            'open_world_data = x_core.open_world_data:main',
        ],
    },
)
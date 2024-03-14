from setuptools import setup

package_name = 'three_way_light'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'three_way_light.pub',


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
            'pub = three_way_light.pub:main',
        ],
    },
)
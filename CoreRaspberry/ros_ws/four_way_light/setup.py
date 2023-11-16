from setuptools import setup

package_name = 'four_way_stoplight'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'four_way_stoplight.serial_com',
        'four_way_stoplight.pub',
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
            'ESP_serial = four_way_stoplight.serial_com:main',
            'Light_pub = four_way_stoplight.pub:main',
        ],
    },
)
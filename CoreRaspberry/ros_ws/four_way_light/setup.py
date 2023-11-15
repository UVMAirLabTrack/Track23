from setuptools import setup

package_name = 'four_way_light'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'four_way_light.ESP_serial',  # Replace with your actual module name
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='Control LED Stoplight via ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ESP_serial = four_way_light.ESP_serial:main',
        ],
    },
)
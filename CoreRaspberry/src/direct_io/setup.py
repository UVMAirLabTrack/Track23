from setuptools import setup

package_name = 'direct_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'direct_io.esp_serial',
        'direct_io.testpub',
        'direct_io.direct_light',
        'direct_io.direct_servo',
        'direct_io.direct_master',

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for main core control of test track',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_serial = direct_io.esp_serial:main',  
            'testpub = direct_io.testpub:main',
            'direct_light = direct_io.direct_light:main',
            'direct_servo = direct_io.direct_servo:main',
            'direct_master = direct_io.direct_master:main',
        ],
    },
)
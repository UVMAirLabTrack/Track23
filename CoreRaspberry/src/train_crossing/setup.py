from setuptools import setup

package_name = 'train_crossing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'train_crossing.pub',


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for controlling a train_crossing using ESP serial communication',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = train_crossing.pub:main',
        ],
    },
)
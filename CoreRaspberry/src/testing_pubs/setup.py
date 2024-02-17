from setuptools import setup
from glob import glob
import os

package_name = 'testing_pubs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={package_name: 'src/' + package_name},
    py_modules=[
    'testing_pubs.circle_odometry',
    'testing_pubs.circle_odometry_mesh',
    'testing_pubs.linear_x_odometry'
    'testing_pubs.color_publisher',
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='ROS 2 package for generating world scenarios in RVIZ',
    license='TODO',
    data_files=[
    ('share/' + package_name + '/config', glob(os.path.join('testing_pubs', 'config', '*.*'))),
    ('share/' + package_name + '/markers', glob(os.path.join('testing_pubs', 'markers', '*.*'))),
    ('share/' + package_name, ['package.xml']),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_odometry = testing_pubs.circle_odometry:main',
            'circle_odometry_mesh = testing_pubs.circle_odometry_mesh:main',
            'linear_x_odometry = testing_pubs.linear_x_odometry:main',
            'color_test = testing_pubs.color_publisher:main',
        ],
    },
)
from setuptools import setup

package_name = 'utility'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'utility.open_world_data',


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian M',
    maintainer_email='iantheterror4@gmail.com',
    description='core function package for the rest of the workspace',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_world_data = utility.open_world_data:main',
        ],
    },
)
from setuptools import setup

package_name = 'core_functions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'core_functions.open_world_data',


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
            'open_world_data = core_functions.open_world_data:main',
        ],
    },
)
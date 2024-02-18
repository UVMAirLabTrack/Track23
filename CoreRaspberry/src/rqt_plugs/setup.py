from setuptools import setup

package_name = 'rqt_plugs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'four_way_rqt= rqt_plugs.four_way_rqt:main',
        ],
        'rosidl_cli.command': [
            'build_package_typesupport_c = rosidl_adapter.cli:BuildPackageTypesupportC',
        ],
        'rosidl_cli.command.verb': [
            'build = rosidl_adapter.verb:Build',
            'info = rosidl_adapter.verb:Info',
        ],
        'rosidl_adapter.typesupport_c': [
            'identifier = rosidl_adapter.typesupport_c:Identifier',
        ],
        'rosidl_adapter.typesupport_c.language': [
            'c = rosidl_adapter.typesupport_c.language:C',
        ],
    },
)
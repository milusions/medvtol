from setuptools import find_packages, setup

package_name = 'airlink'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',"websockets"],
    zip_safe=True,


    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "airlink_websocket_server=airlink.airlink_websocket_server:main",
            "airlink_static_server=airlink.airlink_static_server:main"
        ],
    },
)

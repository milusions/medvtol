from setuptools import find_packages, setup

package_name = 'falcon_inner_loop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "inner_loop=falcon_inner_loop.inner_loop_node:main",
            "pid_setpoint_parser=falcon_inner_loop.pid_setpoint_parser:main"
        ],
    },
)

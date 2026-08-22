from setuptools import find_packages, setup

package_name = 'medvtol_control_panel'

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
    maintainer='random',
    maintainer_email='randompauljs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             "control_panel_ws=medvtol_control_panel.control_panel_ws:main",
              "control_panel_static=medvtol_control_panel.control_panel_static:main",
        ],
    },
)

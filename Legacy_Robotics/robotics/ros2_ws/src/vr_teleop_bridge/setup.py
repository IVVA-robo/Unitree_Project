from glob import glob
from setuptools import find_packages, setup
package_name = 'vr_teleop_bridge'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='unitree',
    maintainer_email='unitree@localhost', description='UDP bridge', license='Apache-2.0',
    entry_points={'console_scripts': ['vr_bridge = vr_teleop_bridge.node:main',]},
)

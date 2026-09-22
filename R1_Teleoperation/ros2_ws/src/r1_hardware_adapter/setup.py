from glob import glob
from setuptools import find_packages, setup


package_name = 'r1_hardware_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description=(
        'Dry-run safety boundary and read-only state monitor for Unitree R1.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r1_hardware_adapter = r1_hardware_adapter.node:main',
            'r1_state_monitor = r1_hardware_adapter.state_monitor:main',
            'r1_head_dry_run = r1_hardware_adapter.head_dry_run:main',
            'r1_locomotion_dry_run = '
            'r1_hardware_adapter.locomotion_dry_run:main',
        ],
    },
)

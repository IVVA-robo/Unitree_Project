from glob import glob

from setuptools import find_packages, setup


package_name = 'r1_teleop_safety'


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
    description='Fail-closed safety interlock and diagnostics for R1 teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r1_safety_supervisor = r1_teleop_safety.supervisor:main',
            'r1_prepare = r1_teleop_safety.prepare:main',
            'r1_teleop_preflight = r1_teleop_safety.preflight:main',
        ],
    },
)

from glob import glob

from setuptools import find_packages, setup


package_name = 'r1_robot_pov'
config_templates = [
    'config/robot_pov.bad-wifi.env',
    'config/robot_pov.exhibition.env',
    'config/robot_pov.mock.env',
    'config/robot_pov.r1.env',
    'config/robot_pov.sim.env',
]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['LICENSE', 'package.xml', 'requirements.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', config_templates),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    python_requires='>=3.10',
    zip_safe=False,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='Offline-first video-only Robot POV server for Unitree R1.',
    license='Apache-2.0',
    url='https://www.unitree.com/',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r1_robot_pov = r1_robot_pov.cli:main',
            'r1_robot_pov_preflight = r1_robot_pov.preflight:main',
        ],
    },
)

from glob import glob

from setuptools import setup


package_name = 'r1_teleoperation_app'


setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='Single safe launch entry point for R1 VR teleoperation simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
)

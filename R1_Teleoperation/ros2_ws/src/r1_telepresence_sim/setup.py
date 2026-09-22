from setuptools import find_packages, setup

package_name = 'r1_telepresence_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/telepresence_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description=(
        'Simulation-only R1 telepresence camera, locomotion adapter and '
        'whole-body trajectory visualizer.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r1_sim_locomotion_adapter = r1_telepresence_sim.locomotion_adapter:main',
            'r1_sim_leg_visualizer = r1_telepresence_sim.leg_visualizer:main',
            'r1_sim_whole_body_planner = r1_telepresence_sim.whole_body_planner:main',
            'r1_stereo_camera_viewer = r1_telepresence_sim.camera_viewer:main',
        ],
    },
)

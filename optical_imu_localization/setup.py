from setuptools import find_packages, setup

package_name = 'optical_imu_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/localization.launch.py',
            'launch/visualization.launch.py',
            'launch/demo_simulation.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/localization.yaml']),
        ('share/' + package_name + '/dsdl/com/hex/equipment/flow', [
            'dsdl/com/hex/equipment/flow/20200.Measurement.uavcan',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Project Team',
    maintainer_email='maintainer@example.com',
    description='Live HereFlow odometry and real-time path visualization.',
    license='Apache-2.0',
    extras_require={
        'can': ['dronecan'],
        'visualization': ['matplotlib'],
    },
    entry_points={
        'console_scripts': [
            'optical_imu_localizer = optical_imu_localization.optical_imu_localizer:main',
            'hereflow_path_plotter = optical_imu_localization.hereflow_path_plotter:main',
            'hereflow_path_demo = optical_imu_localization.path_demo_publisher:main',
        ],
    },
)

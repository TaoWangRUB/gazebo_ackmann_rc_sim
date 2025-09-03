import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files into a launch/ subdirectory
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.yaml')),
        # Install rviz configs
        (os.path.join('share', package_name), glob('resource/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyoung',
    maintainer_email='jalim@ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'velocity_control = px4_offboard.velocity_control:main',
                'pose_control_mavros = px4_offboard.pose_control_mavros:main',
                'velocity_control_mavros = px4_offboard.velocity_control_mavros:main',
                'rc_control_mavros = px4_offboard.rc_control_mavros:main',
                'rc_control_dds = px4_offboard.rc_control_dds:main',
                'vio_to_px4 = px4_offboard.vio_to_px4_dds:main',
        ],
    },
)

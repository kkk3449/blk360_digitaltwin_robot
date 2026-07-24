from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'digital_twin_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caselab',
    maintainer_email='kimsang.m@g.skku.edu',
    description='ammr <-> IsaacSim digital twin bridge',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'dt_state_publisher = digital_twin_bridge.state_publisher:main',
            'dt_goal_relay = digital_twin_bridge.goal_relay:main',
        ],
    },
)

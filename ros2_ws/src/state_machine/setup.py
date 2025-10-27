from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'state_machine'

# Collect data files (launch + params) without hardcoding filenames
launch_files = glob(os.path.join('launch', '*.launch.py'))
param_files  = glob(os.path.join('params', '**', '*.yaml'), recursive=True)
param_files += glob(os.path.join('params', '*.yaml'))  # in case there’s no subfolder

data_files = [
    # ROS 2 ament index
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    # Package manifest
    ('share/' + package_name, ['package.xml']),
    # Launch files
    ('share/' + package_name + '/launch', launch_files),
    # Param files
    ('share/' + package_name + '/params', param_files),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Khedr',
    maintainer_email='you@example.com',
    description='Mission state machine for Nav2 patrol with CPS-triggered autonomous search.',
    license='MIT',
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            'mission_manager = state_machine.mission_manager:main',
        ],
    },
)

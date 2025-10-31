from setuptools import setup, find_packages
from glob import glob

package_name = 'create3_localization_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/maps', glob('config/maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Create3 localization bringup with AMCL and auto initial pose helper',
    license='MIT',
    entry_points={
        'console_scripts': [
            # points to create3_localization_bringup/auto_initial_pose.py:main
            'auto_initial_pose = create3_localization_bringup.auto_initial_pose:main',
        ],
    },
)

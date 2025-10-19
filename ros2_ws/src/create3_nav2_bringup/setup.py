from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'create3_nav2_bringup'

def pkg_files(pattern, subdir):
    return [(os.path.join('share', package_name, subdir), glob(pattern, recursive=True))]

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]
data_files += pkg_files('launch/*.launch.py', 'launch')
data_files += pkg_files('config/*.yaml', 'config')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Nav2 bringup for Create3',
    license='BSD',
    entry_points={'console_scripts': []},
)

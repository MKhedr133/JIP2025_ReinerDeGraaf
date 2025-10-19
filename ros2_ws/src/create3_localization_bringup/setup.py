from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'create3_localization_bringup'

# Helper to collect data files under share/
def package_files(src_glob, dest_subdir):
    return [(os.path.join('share', package_name, dest_subdir), glob(src_glob, recursive=True))]

data_files = [
    # Ament index resource & package.xml
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]

# Install launch/ & config/ (including maps/) into share/<pkg>/
data_files += package_files('launch/*.launch.py', 'launch')
data_files += package_files('config/*.yaml', 'config')
data_files += package_files('config/maps/*', 'config/maps')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed',
    maintainer_email='mkhedr@tudelft.nl',
    description='Bringup for AMCL/SLAM localization on Create3',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # exposes as: ros2 run create3_localization_bringup publish_initialpose
            'publish_initialpose = create3_localization_bringup.publish_initialpose:main',
        ],
    },
)

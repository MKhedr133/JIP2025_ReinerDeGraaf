from setuptools import setup
import os
from glob import glob

package_name = 'create3_localization_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Core package indexes
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'maps'),
        glob('config/maps/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jip25',
    maintainer_email='example@tudelft.nl',
    description='Localization-only bringup for Create3 + SLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

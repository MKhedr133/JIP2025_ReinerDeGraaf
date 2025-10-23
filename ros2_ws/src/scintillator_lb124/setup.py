from setuptools import find_packages, setup

package_name = 'scintillator_lb124'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/scintillator.launch.py']),
        (f'share/{package_name}/csv', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Khedr',
    maintainer_email='mkhedr@tudelft.nl',
    description='ROS 2 nodes for LB-124 scintillator: raw serial reader, CPS filter, CSV logger',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scintillator_raw_node = scintillator_lb124.scintillator_raw:main',
            'scintillator_filtered_node = scintillator_lb124.scintillator_filtered:main',
            'scintillator_csv_node = scintillator_lb124.scintillator_csv:main',
            'cps_statistics = scintillator_lb124.cps_statistics:main',
        ],
    },
)
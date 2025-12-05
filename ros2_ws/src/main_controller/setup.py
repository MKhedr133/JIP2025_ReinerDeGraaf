from setuptools import find_packages, setup
import os

package_name = 'main_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), ['config/autonomy_params.yaml']),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/main_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joost',
    maintainer_email='jevanbusschbach@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_controller = main_controller.main_controller:main',
            'basic = main_controller.autonomous_controller:main',
        ],
    },
)
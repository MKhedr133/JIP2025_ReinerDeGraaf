from setuptools import setup

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission_autonomy.launch.py']),
        ('share/' + package_name + '/params', ['params/state_machine.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Khedr',
    maintainer_email='you@example.com',
    description='Mission state machine for Nav2 patrol + CPS trigger → autonomous controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = state_machine.mission_manager:main',
        ],
    },
)

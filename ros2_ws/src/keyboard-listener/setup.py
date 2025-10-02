from setuptools import find_packages, setup

package_name = 'keyboard_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'pynput'],
    zip_safe=True,
    maintainer='joost',
    maintainer_email='jevanbusschbach@gmail.com',
    description='Node that is responsible for listening to keyboard inputs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_listener = keyboard_listener.keyboard_listener:main'
        ],
    },
)

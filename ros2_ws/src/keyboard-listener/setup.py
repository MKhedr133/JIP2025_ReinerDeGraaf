from setuptools import find_packages, setup

package_name = 'keyboard-listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joost',
    maintainer_email='jevanbusschbach@gmail.com',
    description='Node that is responsible for listening to keyboard inputs',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'KeyboardListener = keyboard-listener.KeyboardListener:main'
        ],
    },
)

from setuptools import find_packages, setup
import os

package_name = 'tb3_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [
            os.path.join('resource', package_name)
        ]),
        (os.path.join('share', package_name, 'launch'), [
            os.path.join('launch', 'gz_tb3_house.launch.py'),
            os.path.join('launch', 'gz_rviz_tb3_house.launch.py'),
        ]),
        (os.path.join('share', package_name, 'worlds'), [
            os.path.join('worlds', 'turtlebot3_house.world')
        ]),
        (os.path.join('share', package_name, 'rviz'), [
            os.path.join('rviz', 'tb3_gazebo.rviz')
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayden',
    maintainer_email='haydenmcameron@proton.me',
    description='Particle Filter Based localization for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_sensor_models = tb3_localization.motion_sensor_models:main',
        ],
    },
)

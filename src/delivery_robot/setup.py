from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'delivery_robot'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        # Đăng ký package với ROS 2
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # Cài package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Cài các file launch
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        # Cài các file world
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')
        ),

        # Cài model của robot
        (
            os.path.join('share', package_name, 'models', 'delivery_robot'),
            glob('models/delivery_robot/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='lap',
    maintainer_email='lap@todo.todo',

    description='Delivery robot simulation',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'auto_controller = delivery_robot.keyboard_controller:main',
            'a_star_node = delivery_robot.a_star_node:main',
            'dwa_node = delivery_robot.dwa_node:main',
        ],
    },
)
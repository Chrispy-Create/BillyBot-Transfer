import os
from glob import glob
from setuptools import setup

package_name = 'billybot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billybot',
    maintainer_email='billybot@todo.todo',
    description='BillyBot bringup package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_odom    = billybot_bringup.open_loop_odom:main',
            'ultrasonic_bridge = billybot_bringup.ultrasonic_bridge:main',
            'safety_node       = billybot_bringup.safety_node:main',
        ],
    },
)

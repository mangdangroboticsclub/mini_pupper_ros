from setuptools import setup
import os
from glob import glob

package_name = 'mini_pupper_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='kishan',
    maintainer_email='kishangrewal06@gmail.com',
    description='Mini Pupper vision-based tracking system',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = mini_pupper_tracking.main:main',
            'movement_node = mini_pupper_tracking.movement_node:main',
            'camera_visualisation_node = mini_pupper_tracking.camera_visualisation_node:main',
        ],
    },
)
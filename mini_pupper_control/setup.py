from setuptools import setup
from glob import glob
import os

package_name = 'mini_pupper_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name,
              f'{package_name}.StanfordQuadruped',
              f'{package_name}.StanfordQuadruped.pupper',
              f'{package_name}.StanfordQuadruped.src'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools', 'transforms3d', 'numpy'],
    zip_safe=True,
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='Mini pupper motor control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_to_servo_controller =\
                  mini_pupper_control.vel_to_servo_controller:main',
        ],
    },
)

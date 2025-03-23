from setuptools import setup
import os
from glob import glob

package_name = 'stanford_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='A ROS 2 Python package for the StanfordController node.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanford_controller_node = stanford_controller.stanford_controller_node:main',
            'twist_to_command_node = stanford_controller.twist_to_command_node:main'
        ],
    },
)

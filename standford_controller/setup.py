from setuptools import setup
import os
from glob import glob

package_name = 'standford_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cullensun',
    maintainer_email='sunhongshuai@gmail.com',
    description='A ROS 2 Python package for the StandfordController node.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanford_controller = standford_controller:main'
        ],
    },
)

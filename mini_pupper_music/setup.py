from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_pupper_music'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cullensun',
    maintainer_email='sunhongshuai@gmail.com',
    description='This package plays a cool sound, especially while dancing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = mini_pupper_music.music_server:main',
        ],
    },
)

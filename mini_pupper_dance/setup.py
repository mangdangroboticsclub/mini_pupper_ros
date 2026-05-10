from setuptools import setup
from glob import glob
import os

package_name = 'mini_pupper_dance'

setup(
    name=package_name,
    version='0.0.1',
    packages=['mini_pupper_dance'],
    package_dir={
        'mini_pupper_dance': 'mini_pupper_dance'
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mangdang',
    maintainer_email='mangdang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini_pupper_dance = mini_pupper_dance.mini_pupper_dance:main'
        ]
    }
)

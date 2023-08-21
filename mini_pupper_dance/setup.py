from setuptools import setup
import os
from glob import glob

package_name = 'mini_pupper_dance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
            'service = mini_pupper_dance.dance_server:main',
            'client = mini_pupper_dance.dance_client:main',
            'pose_controller = mini_pupper_dance.pose_controller:main'
        ],
    },
)

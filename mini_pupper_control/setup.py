from setuptools import setup
from glob import glob
import os

package_name = 'mini_pupper_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MangDang',
    author_email='fae@mangdang.net',
    maintainer='Yunlong Feng, Zhengxiao Han',
    maintainer_email='ericc_feng@qq.com, thefoxfoxfox@outlook.com',
    description='The mini_pupper_control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_interface = mini_pupper_control.servo_interface:main',
            'display_interface = mini_pupper_control.display_interface:main'
        ],
    },
)

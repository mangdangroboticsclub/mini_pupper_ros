from setuptools import setup
from glob import glob
import os

package_name = 'mini_pupper_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, package_name), glob(package_name+'/*.py')),
        (os.path.join('share', package_name, package_name, 'src'), glob(package_name+'/src/*.py')),
        (os.path.join('share', package_name, package_name, 'pupper'), glob(package_name+'/pupper/*.py')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'Adafruit_GPIO'), glob(package_name+'/Mangdang/Adafruit_GPIO/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'EEPROM'), glob(package_name+'/Mangdang/EEPROM/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'Example', 'display'), glob(package_name+'/Mangdang/Example/display/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'FuelGauge'), glob(package_name+'/Mangdang/FuelGauge/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'IO_Configuration'), glob(package_name+'/Mangdang/IO_Configuration/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'LCD'), glob(package_name+'/Mangdang/LCD/*.py')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'LCD', 'cartoons'), glob(package_name+'/Mangdang/LCD/cartoons/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'PWMController'), glob(package_name+'/Mangdang/PWMController/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'stuff'), glob(package_name+'/Mangdang/stuff/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'System'), glob(package_name+'/Mangdang/System/*')),
        (os.path.join('share', package_name, package_name, 'Mangdang', 'Tools'), glob(package_name+'/Mangdang/Tools/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MangDang',
    author_email='fae@mangdang.net',
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='The mini_pupper_driver package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_to_servo_control_interface = mini_pupper_driver.vel_to_servo_control_interface:main',
        ],
    },
)

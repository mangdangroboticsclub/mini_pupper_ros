# Copyright 2025 Kishan Grewal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import setup
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
            'webcam_node = mini_pupper_tracking.webcam_node:main',
        ],
    },
)

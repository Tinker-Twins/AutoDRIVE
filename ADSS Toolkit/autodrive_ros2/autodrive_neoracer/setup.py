################################################################################

# Copyright (c) 2026, Tinker Twins, AutoDRIVE Ecosystem
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_neoracer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # Config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz config files
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')), # Map metadata files
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')), # Map image files
    ],
    install_requires=['setuptools'],
    zip_safe=True,maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem ROS 2 Package for NeoRacer',
    license='BSD',
    entry_points={
        'console_scripts': [
            'autodrive_bridge = autodrive_neoracer.autodrive_bridge:main',
            'autodrive_watchdog = autodrive_neoracer.autodrive_watchdog:main',
            'teleop_keyboard = nodes.teleop_keyboard:main',
        ],
    },
)

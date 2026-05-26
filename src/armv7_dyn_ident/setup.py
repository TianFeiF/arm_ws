# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
from glob import glob

from setuptools import setup

package_name = 'armv7_dyn_ident'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TianFeiF',
    maintainer_email='chunyvtian@gmail.com',
    description='Dynamics parameter identification (static gravity calibration) for armv7.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect = armv7_dyn_ident.collect_node:main',
            'identify = armv7_dyn_ident.identify:main',
        ],
    },
)

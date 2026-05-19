# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup

package_name = 'armv7_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TianFeiF',
    maintainer_email='chunyvtian@gmail.com',
    description='Runnable examples for armv7.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world    = armv7_examples.hello_world:main',
            'teach_playback = armv7_examples.teach_playback:main',
            'pose_grid      = armv7_examples.pose_grid:main',
        ],
    },
)

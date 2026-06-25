# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup

package_name = 'armv7_rqt'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # rqt plugin discovery marker (ament_python does not auto-create it).
        ('share/ament_index/resource_index/rqt_gui__pluginlib__plugin',
         ['resource_marker/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TianFeiF',
    maintainer_email='chunyvtian@gmail.com',
    description='Unified rqt control panel for armv7 (mode switch + impedance tuning).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

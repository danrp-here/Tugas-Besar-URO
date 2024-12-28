from setuptools import setup
import os
from glob import glob

package_name = 'plane_kinematics_algorithm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodding',
    maintainer_email='iniakundanny@gmail.com',
    description='rOBots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_controller = plane_kinematics_algorithm.kinematics_controller:main',
        ],
    },
)


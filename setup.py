import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multicam_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gh4',
    maintainer_email='aura.team.gh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "data_collection_node = multicam_pointcloud.data_collector:main",
            "luxonis_multicam_node = multicam_pointcloud.luxonis_multicam:main",
            "mcpc_node = multicam_pointcloud.mcpc_controller:main"
        ],
    },
)

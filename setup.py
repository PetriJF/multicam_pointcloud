from setuptools import find_packages, setup
import os
import logging

package_name = 'multicam_pointcloud'

# Set up logging to a known location
log_file_path = os.path.expanduser('~/setup_debug.log')
logging.basicConfig(filename=log_file_path, level=logging.DEBUG, filemode='w')

# Start logging
logging.debug("Running setup.py for package: %s", package_name)

# Define the data files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), [os.path.join('multicam_pointcloud', 'config', 'rs_405_camera_config.yaml')]),
]

# Log the data files to be installed
logging.debug("Data files to be installed:")
for dest, sources in data_files:
    for source in sources:
        full_source_path = os.path.abspath(source)
        full_dest_path = os.path.join(os.path.abspath('install'), dest, os.path.basename(source))
        logging.debug("Source: %s -> Destination: %s", full_source_path, full_dest_path)

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Petri',
    maintainer_email='jamespetri28@gmail.com',
    description='Package for capturing and processing multi-camera RealSense data in ROS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_subscriber = multicam_pointcloud.cam_subscriber:main',
            'mcpc_controller = multicam_pointcloud.mcpc_controller:main'
            'mcpc_node = multicam_pointcloud.mcpc_node:main'
        ],
    },
)

# End logging
logging.debug("Finished running setup.py for package: %s", package_name)
logging.debug("Debug information written to: %s", log_file_path)

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os, yaml

class MyNode(Node):
    def __init__(self):
        super().__init__('runner_node')
        self.get_logger().info('Test')

        # Relevant directory and file names
        self.directory_ = os.path.join(
            get_package_share_directory('map_handler'),
            'config'
        )
        self.active_map_file_ = 'active_map.yaml'

        self.form_sequence('RING_7_3', 100)

    def form_sequence(self, pattern: str, steps_mm: int):
        plant_grid = self.form_grid(pattern)
        self.save_to_yaml(plant_grid, '/home/farmbotdev/FarmBot_ROS2/src/multicam_pointcloud/multicam_pointcloud/', 'plants.yaml')


    def form_grid(self, pattern: str) -> dict:
        # Load the map information from the config file
        map_instance = self.retrieve_map(self.directory_, self.active_map_file_)

        # Initialize a dictionary to store tomato plants
        tomato_plants = {}

        # Extract and sort tomato plants
        for plant_id, plant_info in map_instance['plant_details']['plants'].items():
            if plant_info['identifiers']['plant_name'] == 'Tomato':
                plant = {
                    'name': plant_info['identifiers']['plant_name'],
                    'index': plant_info['identifiers']['index'],
                    'position': plant_info['position'],
                    'row_last': False  # default value, to be updated later
                }
                tomato_plants[plant_info['identifiers']['index']] = plant

        # Sort the list of dictionaries based on 'position'['x']. Note that the dictionary read above is
        # converted to a list of dictionaries
        sorted_tomato_plants_list = sorted(list(tomato_plants.values()), key=lambda x: x['position']['x'])
        
        # Get the pattern dimensions
        _, N, M = pattern.split('_')
        N = int(N)
        M = int(M)
        
        # Adding the first row of the ring pattern
        temp = sorted(sorted_tomato_plants_list[0:M], key=lambda y: y['position']['y'])
        temp[M-1]['row_last'] = True

        if pattern.startswith('RING_'):
            # Adding the middle N-2 rows of the ring pattern
            for row in range(0, N-2):
                temp.extend(sorted(sorted_tomato_plants_list[M+row*2:M+row*2+2], key=lambda y: y['position']['y']))
                temp[M+row*2+1]['row_last'] = True
            
            # Adding the Nth row of the ring pattern
            temp.extend(sorted(sorted_tomato_plants_list[M+(N-2)*2: M*2+(N-2)*2], key=lambda y: y['position']['y']))
            temp[M*2+(N-2)*2 - 1]['row_last'] = True
        elif pattern.startswith('GRID_'):
            # Adding the middle N-2 rows of the grid pattern
            for row in range(0, N-2):
                temp.extend(sorted(sorted_tomato_plants_list[M+row*M:M+row*M+M], key=lambda y: y['position']['y']))
                temp[M*2+row*M-1]['row_last'] = True
            # Adding the Nth row of the grid pattern
            temp.extend(sorted(sorted_tomato_plants_list[M+(N-2)*M: M+(N-2)*M+M], key=lambda y: y['position']['y']))
            temp[-1]['row_last'] = True
        else:
            self.get_logger().error('Pattern not recognized!')
        
        # Convert sorted list back to dictionary with adjusted keys starting from 1
        return {index: plant for index, plant in enumerate(temp, start=1)}
    

    def retrieve_map(self, directory = '', file_name = ''):
        '''
        Attempts to retrieve the map configuration file from memory.
        If it fails, it either means that the file was deleted or the
        current run is a fresh run. This node will not work on a fresh
        run unless plants are added in manually.

        Args:
            directory {String}: The share directory the yaml files are located at
            file_name1 {String}: The active config (i.e. the one from memory)
        '''
        active_config = os.path.join(directory, file_name)
        if os.path.exists(active_config):
            self.get_logger().info('Initialized map from previous run!')
            return self.load_from_yaml(directory, file_name)
        else:
            self.get_logger().error('Previous map info could not be found! Unless you have a back-up, previous items need to be re-added')
            return
        
    def load_from_yaml(self, path = '', file_name = ''):
        '''
        Leads a dictionary from a yaml file in the share directory

        Args:
            path {String}: The share directory the yaml files are located at
            file_name {String}: The active config (i.e. the one from memory)
        '''
        if path == '':
            self.get_logger().warn('Path not set for retrieving the parameter config file')
            return
        if file_name == '':
            self.get_logger().warn('Parameter Config File name not set')
            return
        if not os.path.exists(path):
            self.get_logger().warn('File path is invalid')
            return
        
        with open(os.path.join(path, file_name), 'r') as yaml_file:
            loaded_data = yaml.safe_load(yaml_file)
            if isinstance(loaded_data, dict):
                return loaded_data
            else:
                self.get_logger().warn('Invalid YAML file format..')

    def save_to_yaml(self, data: dict, path = '', file_name = '', create_if_empty = False):
        '''
        Saves a dictionary to a yaml file in the share directory. If the file exists already,
        it updates it.

        Args:
            path {String}: The share directory the yaml files are located at
            file_name {String}: The active config (i.e. the one from memory)
        '''
        if not isinstance(data, dict):
            self.get_logger().warn('Parsed dictionary data is not of type dictionary')
            return
        if path == '':
            self.get_logger().warn('Path not set for retrieving the parameter config file')
            return
        if file_name == '':
            self.get_logger().warn('Parameter Config File name not set')
            return
        if not create_if_empty and not os.path.exists(path):
            self.get_logger().warn('File path is invalid')
            return
        
        if create_if_empty and not os.path.exists(path):
            self.get_logger().info('Creating the active map configuration file..')
            os.makedirs(os.path.dirname(os.path.join(path, file_name)), exist_ok=True)

        self.get_logger().info(f'Saving current parameter configuration at {os.path.join(path, file_name)}')
            
        with open(os.path.join(path, file_name), 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style = False)
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

import math
import os, yaml

class PointCloudController(Node):
    '''
    Acts as the main controller for forming the command sequence and controlling
    the multipoint cloud image acquisition software.
    '''
    def __init__(self):
        super().__init__('mcpc_controller')

        # Relevant directory and file names
        self.directory_ = os.path.join(
            get_package_share_directory('map_handler'),
            'config'
        )
        self.active_map_file_ = 'active_map.yaml'

        self.msg_ = String()
        self.final_sequence = ''
        self.input_pub_ = self.create_publisher(String, 'keyboard_topic', 10)

        self.sequencer_pub_ = self.create_publisher(String, 'sequencer', 10)

        self.get_logger().info(
        '''
            This is the main control point for the 3-Camera setup. This system
            can only be controlled from here (can NOT be controlled from the
            keyboard controller).\n
            List of commands:
                * 'FORM' = creates the sequence needed to control everything
                * 'PRINT' = prints out the formed sequence
                * 'RUN' = sends the sequence to the sequence manager
                * Commands that are the same as in the keyboard_controller:
                    - 'e'
                    - 'E'
                    - 'C_0'
                    - 'CONF'
                    - 'H_0'
        ''')

    def controller(self):
        valid_cmds = [
            'FORM',
            'PRINT',
            'RUN',
            'e', 'E', 'C_0', 'CONF', 'H_0'
        ]
        
        user_input = input('\nEnter command: ')

        if user_input in valid_cmds:
            if user_input == 'FORM':
                self.final_sequence = self.form_sequence('RING_7_3', 200, 450)
            elif user_input == 'PRINT':
                self.get_logger().info(self.final_sequence)
            elif user_input == 'RUN':
                self.msg_.data = self.final_sequence
                self.sequencer_pub_.publish(self.msg_)
                self.get_logger().info("Running the 3-Camera Imager sequence!")
            elif user_input in ['e', 'E', 'C_0', 'CONF', 'H_0']:
                self.msg_.data = user_input
                self.input_pub_.publish(self.msg_)
            else:
                self.get_logger().error('Not Implemeted!')
        else:
            self.get_logger().warning('Command not recognized')


    def form_sequence(self, pattern: str, steps_mm: int, d_offset: int) -> str:
        '''
        Forms the sequence for capturing images of each plant row at given
        intervals
        Args:
            pattern {str}: pattern to form the grid by
            steps_mm {int}: increment between capture positions
            d_offset {offset between toolhead center and horizontal camera}
        '''
        plant_grid, max_x, max_y = self.form_grid(pattern)

        def modify_coords(coords, value, x_min, x_max, y_min, y_max):
            '''
            Maps the coordinates based on the map dimensions and d-offset.
            '''
            return [
                [
                    min(max(x + value[0], x_min), x_max), 
                    min(max(y + value[1], y_min), y_max)
                ] for x, y in coords
            ]


        def get_row_sequence(coords: list, step: int) -> str:
            '''
            Forms the sequence for the coordinates by moving from point to point
            in the given step increments.
            '''
            sub_sequence = ''
            
            if len(coords) < 2:
                return ''
            
            x_1, y_1 = coords[0]
            x_2, y_2 = coords[1]
            
            # Calculate the Euclidean distance between the two points
            dist = math.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2)
            # Initialize travel distance and step counter
            travel = 0
            n = 0
            
            while travel < dist:
                # Calculate current step's x and y positions
                x_curr = x_1 + (n * step / dist) * (x_2 - x_1)
                y_curr = y_1 + (n * step / dist) * (y_2 - y_1)
                            
                # Move to the given coordinate
                sub_sequence += 'CC_3_Cam\n'
                sub_sequence += f'{x_curr} {y_curr} {0.0}\n'
                # Take the 3 pictures
                sub_sequence += 'VC_3_Cam\n'
                sub_sequence += 'M_CAM_TAKE\n'

                # Increment travel distance and step counter
                travel += step
                n += 1
            
            # Print the last point if it hasn't been printed yet
            if travel != dist:
                sub_sequence += 'CC_3_Cam\n'
                sub_sequence += f'{x_curr} {y_curr} {0.0}\n'
                sub_sequence += 'VC_3_Cam\n'
                sub_sequence += 'M_CAM_TAKE\n'
            
            # Recursively call the function for the remaining points
            return sub_sequence + get_row_sequence(coords[1:], step)

        first_row_key = -1
        coords_to_hit = []

        sequence = ''
        for key in plant_grid:
            if first_row_key == -1:
                first_row_key = int(key)

            coords_to_hit.append([plant_grid[key]['position']['x'], plant_grid[key]['position']['y']])
            if plant_grid[key]['row_last']:
                #self.get_logger().info('Reached the end')

                # Get the coordinates for capturing the images
                coords = modify_coords(coords_to_hit, [-d_offset, 0.0], 0.0, max_x, 0.0, max_y)
                # Rotate the servo in a safe area
                sequence += f'CC_3_Cam\n{coords[0][0]} {0.0} 0.0\n'
                # Rotate the servo to the left side of the fence
                sequence += 'SC_3_Cam\n180.0\n'
                # Record the plants on on side
                sequence += get_row_sequence(coords, steps_mm)
                # Get the coordinates for capturing the images
                coords = modify_coords(coords_to_hit, [d_offset, 0.0], 0.0, max_x, 0.0, max_y)
                # Rotate the servo in a safe area
                sequence += f'CC_3_Cam\n{coords[0][0]} {0.0} 0.0\n'
                # Rotate the servo to the right side of the fence
                sequence += 'SC_3_Cam\n0.0\n'
                # Record the plants on the other side
                sequence += get_row_sequence(coords, steps_mm)

                #self.get_logger().info(str(coords_to_hit))
                coords_to_hit.clear()
                first_row_key = -1
        
        return sequence


    def form_grid(self, pattern: str) -> dict:
        '''
        Based on the information from the active map instance,
        a grid plan for going from plant to plant according to
        the given pattern is formed.

        Args:
            pattern {str}: pattern to follow (GRID or RING)
        '''
        
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
        return {index: plant for index, plant in enumerate(temp, start=1)}, map_instance['map_reference']['x_len'], map_instance['map_reference']['y_len']
    

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
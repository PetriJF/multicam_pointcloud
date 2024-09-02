#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

import time
import threading
from datetime import datetime
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

        config_directory = os.path.join(get_package_share_directory('multicam_pointcloud'), 'config')
        system_config_file = 'mcpc_system_config.yaml'
        system_config_data = self.load_from_yaml(config_directory, system_config_file)

        self.servo_0_pos = system_config_data['servo_left_rot']
        self.servo_180_pos = system_config_data['servo_right_rot']
        self.ROW_Y_SWITCH_OFFSET = system_config_data['row_y_switch_offset']

        self.msg_ = String()
        self.final_sequence = ''

        self.daily_iter_ = -1
        self.timer_ = self.create_timer(60.0, self.start_reading)

        self.input_pub_ = self.create_publisher(String, 'keyboard_topic', 10)
        self.sequencer_pub_ = self.create_publisher(String, 'sequencer', 10)

        self.get_logger().info(
        '''
            This is the main control point for the 3-Camera setup. This system
            can only be controlled from here (can NOT be controlled from the
            keyboard controller).\n
            List of commands:
                * 'FORM I J DIST SIDE' = creates the sequence needed to control everything
                * 'FORM I J DIST SIDE D' = same as above but you can specify the D Len
                * 'FORM_TYPE I J DIST SIDE D' = same as above but you can specify the plant arrangement type (GRID or RING)
                * 'PRINT' = prints out the formed sequence
                * 'RUN' = sends the sequence to the sequence manager
                * 'RUN_C N' = runs the sequence the N times selected
                * Commands that are the same as in the keyboard_controller:
                    - 'e'
                    - 'E'
                    - 'C_0'
                    - 'CONF'
                    - 'H_0'
        ''')

    def start_reading(self):
        now = datetime.now().time()
        current_time = now.strftime('%H:%M')
        if self.daily_iter_ == -1:
            pass
        elif self.daily_iter_ == 2:
            if current_time == '7:50' or current_time == '8:00':
                self.msg_.data = 'H_0'
                self.input_pub_.publish(self.msg_)
                self.get_logger().info('Homing the robot!')
            elif current_time == '8:00' or current_time == '14:10':
                self.msg_.data = self.final_sequence
                self.sequencer_pub_.publish(self.msg_)
                self.get_logger().info('Running the 3-Camera Imager sequence!')
            

    def controller(self):
        valid_cmds = [
            'FORM',
            'FORM_TYPE',
            'PRINT',
            'RUN',
            'RUN_C',
            'e', 'E', 'C_0', 'CONF', 'H_0'
        ]

        user_input = input('\nEnter command: ')
        user_input_split = user_input.split(' ')

        if user_input_split[0] in valid_cmds:
            if user_input_split[0] == 'FORM' and len(user_input_split) == 5:
                rows = user_input_split[1]
                cols = user_input_split[2]
                incr = int(user_input_split[3])
                turn = bool(int(user_input_split[4]))
                self.final_sequence = self.form_sequence(f'RING_{rows}_{cols}', steps_mm = incr, turn_side = turn, d_offset = 395.7)
            elif user_input_split[0] == 'FORM' and len(user_input_split) == 6:
                rows = user_input_split[1]
                cols = user_input_split[2]
                incr = int(user_input_split[3])
                turn = bool(int(user_input_split[4]))
                dOff = float(user_input_split[5])
                self.final_sequence = self.form_sequence(f'RING_{rows}_{cols}', steps_mm = incr, turn_side = turn, d_offset = dOff)
            elif user_input_split[0] == 'FORM_TYPE' and len(user_input_split) == 7:
                shape = user_input_split[1]
                rows = user_input_split[2]
                cols = user_input_split[3]
                incr = int(user_input_split[4])
                turn = bool(int(user_input_split[5]))
                dOff = float(user_input_split[6])
                if incr > 0:
                    self.final_sequence = self.form_sequence(f'{shape}_{rows}_{cols}', steps_mm = incr, turn_side = turn, d_offset = dOff, plant = 'Oat')
                else:
                    self.final_sequence = self.form_sequence(f'{shape}_{rows}_{cols}', steps_mm = incr, turn_side = turn, d_offset = dOff, plant = 'Oat', sequence_points = True)
            elif user_input_split[0] == 'PRINT':
                self.get_logger().info(self.final_sequence)
            elif user_input_split[0] == 'RUN':
                self.msg_.data = self.final_sequence
                self.sequencer_pub_.publish(self.msg_)
                self.get_logger().info('Running the 3-Camera Imager sequence!')
            elif user_input_split[0] == 'RUN_C':
                self.daily_iter_ = int(user_input_split[1])
            elif user_input_split[0] in ['e', 'E', 'C_0', 'CONF', 'H_0']:
                self.msg_.data = user_input
                self.input_pub_.publish(self.msg_)
            else:
                self.get_logger().error('Not Implemeted!')
        else:
            self.get_logger().warning('Command not recognized')


    def form_sequence(self, pattern: str, turn_side: bool, d_offset: int, steps_mm: int = 0, servo_pin:int = 4, plant:str = 'Tomato', sequence_points:bool = False) -> str:
        '''
        Forms the sequence for capturing images of each plant row at given intervals.
        Args:
            pattern   {str} : pattern to form the grid by.
            steps_mm  {int} : increment between capture positions.
            d_offset  {int} : offset between toolhead center and horizontal camera.
            turn_side {bool}: side on which the unit should attempt to move between rows.
            servo_pin {int} : the pin to which the servo is connected.
            plant     {str} : the plant type we are looking for in the active map.
        '''
        plant_grid, max_x, max_y = self.form_grid(pattern, plant_type=plant)

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

        def get_row_sequence(coords: list, step: int, no_travel:bool = False) -> str:
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
            
            if not no_travel:
                while travel <= dist:
                    # Calculate current step's x and y positions
                    x_curr = int(x_1 + (n * step / dist) * (x_2 - x_1))
                    y_curr = int(y_1 + (n * step / dist) * (y_2 - y_1))
                                
                    # Move to the given coordinate
                    sub_sequence += 'CC_3_Cam\n'
                    sub_sequence += f'{x_curr} {y_curr} {0}\n'
                    # Take the 3 pictures
                    sub_sequence += 'VC_3_Cam\n'
                    sub_sequence += 'M_CAM_TAKE\n'
                    # sub_sequence += f"TD_TICK_DELAY\nT{170}\n"

                    # Increment travel distance and step counter
                    travel += step
                    n += 1
                
                # Print the last point if it hasn't been printed yet
                if travel != dist:
                    sub_sequence += 'CC_3_Cam\n'
                    sub_sequence += f'{x_curr} {y_curr} {0}\n'
                    sub_sequence += 'VC_3_Cam\n'
                    sub_sequence += 'M_CAM_TAKE\n'
                    # sub_sequence += f"TD_TICK_DELAY\nT{170}\n"
            else:
                sub_sequence += 'CC_3_Cam\n'
                sub_sequence += f'{x_1} {y_1} {0}\n'
                # Take the 3 pictures
                sub_sequence += 'VC_3_Cam\n'
                sub_sequence += 'M_CAM_TAKE\n'
                # sub_sequence += f"TD_TICK_DELAY\nT{170}\n"

                if len(coords) == 2:
                    sub_sequence += 'CC_3_Cam\n'
                    sub_sequence += f'{x_2} {y_2} {0}\n'
                    # Take the 3 pictures
                    sub_sequence += 'VC_3_Cam\n'
                    sub_sequence += 'M_CAM_TAKE\n'
                    # sub_sequence += f"TD_TICK_DELAY\nT{170}\n"
            
            # Recursively call the function for the remaining points
            return sub_sequence + get_row_sequence(coords[1:], step, no_travel=no_travel)

        first_row_key = -1
        coords_to_hit = []

        sequence = ''
        for key in plant_grid:
            if first_row_key == -1:
                first_row_key = int(key)

            coords_to_hit.append([plant_grid[key]['position']['x'], plant_grid[key]['position']['y']])
            if plant_grid[key]['row_last']:
                # Get the coordinates for capturing the images
                coords = modify_coords(coords_to_hit, [-d_offset, 0.0], 0.0, max_x, 0.0, max_y)
                # Rotate the servo in a safe area
                sequence += f'CC_3_Cam\n{int(coords[0][0])} {0} 0\n'
                # Rotate the servo to the left side of the fence
                sequence += f'SC_3_Cam\n{servo_pin} {self.servo_180_pos}\n'
                # Record the plants on one side
                sequence += (get_row_sequence(coords, steps_mm) if not sequence_points else get_row_sequence(coords, steps_mm, no_travel = True))
                
                # Move the the other side of the row
                if not turn_side:
                    # Rotate the servo in a safe area
                    sequence += f'CC_3_Cam\n{int(coords[0][0])} {0} 0\n'
                    # Rotate the servo to the right side of the fence
                    sequence += f'SC_3_Cam\n{servo_pin} {self.servo_0_pos}\n'
                    # Get the coordinates for capturing the images
                    coords = modify_coords(coords_to_hit, [d_offset, 0.0], 0.0, max_x, 0.0, max_y)
                    # Move to the other side
                    sequence += f'CC_3_Cam\n{int(coords[0][0])} {0} 0\n'
                    # Record the plants on the other side
                    sequence += (get_row_sequence(coords, steps_mm) if not sequence_points else get_row_sequence(coords, steps_mm, no_travel = True))
                else: 
                    # Move as close to the end of the y-axis as possible
                    sequence += f'CC_3_Cam\n{int(coords[0][0])} {int(max_y - self.ROW_Y_SWITCH_OFFSET)} 0\n'
                    # Get the coordinates for capturing the images
                    coords = modify_coords(coords_to_hit, [d_offset, 0.0], 0.0, max_x, 0.0, max_y)
                    # Move to the other side
                    sequence += f'CC_3_Cam\n{int(coords[0][0])} {int(max_y - self.ROW_Y_SWITCH_OFFSET)} 0\n'
                    # Move to the start of the row
                    sequence += f'CC_3_Cam\n{int(coords[0][0])} {0} 0\n'
                    # Rotate the servo to the right side of the fence
                    sequence += f'SC_3_Cam\n{servo_pin} {self.servo_0_pos}\n'
                    # Record the plants on the other side
                    sequence += (get_row_sequence(coords, steps_mm) if not sequence_points else get_row_sequence(coords, steps_mm, no_travel = True))
                
                coords_to_hit.clear()
                first_row_key = -1
        
        return sequence

    def form_grid(self, pattern: str, plant_type:str = 'Tomato') -> dict:
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
            if plant_info['identifiers']['plant_name'] == plant_type:
                plant = {
                    'name': plant_info['identifiers']['plant_name'],
                    'index': plant_info['identifiers']['index'],
                    'position': plant_info['position'],
                    'row_last': False  # default value, to be updated later
                }
                tomato_plants[plant_info['identifiers']['index']] = plant

        # Sort the list of dictionaries based on 'position'['x']
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
            
def main(args = None):
    # MCPC Python Node installer
    rclpy.init(args = args)
    node = PointCloudController()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    try:
        while rclpy.ok():
            node.controller()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == "__main__":
    main()

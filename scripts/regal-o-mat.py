#!/usr/bin/env python3

import yaml
import sys

"""
Regal-O-Mat:

Generates ecwm simple_geometry.yaml and affordances.yaml representation of a shelf based on values given via the command
line at runtime or a yaml config.

You have to run this script from within the "scripts" dir!

Following assumptions are made:
- All vertical boards do have the same thickness
- All horizontal boards do have the same thickness
- All shelf boards do have the same depth
"""

class ShelfGenerator():
    """
    Makes the shelf shelfing.
    """
    def __init__(self):
        self.shelf_height = 0.0
        self.shelf_width = 0.0
        self.shelf_depth = 0.0
        self.vertical_board_thickness = 0.0
        self.horizontal_board_thickness = 0.0
        self.shelf_board_depth = 0.0
        self.number_shelf_boards = 0
        self.clearances_above_shelf_boards = []
        self.height_first_board = 0.0

        self.do_create_affordances = False
        self.do_pad_affordances = False
        self.affordance_padding = {'x': 0.0, 'y': 0.0}
        self.create_affordances_for = []
        self.affordance_max_z = 0.0

        self.simple_geometry = {
            'model_spec': 1,
            'primitives': []
        }

        self.affordances = {
            'storages' : []
        }

        self.CONFIG = None

    def run(self):
        self._get_user_input()
        self._convert_to_meters()
        self._generate()
        self._export_to_yaml()

    def run_test(self):
        self._test_input()
        self._convert_to_meters()
        self._generate()
        self._export_to_yaml()

    def _get_user_input(self):
        self.do_use_config = input("Use shelf_config.yaml? (y/n): ").lower().startswith('y')
        if self.do_use_config:
            self._read_config()
            self._load_config()
            return
        print("=== General dimensions of the complete shelf ===")
        self.shelf_height = float(input("Enter height of shelf in [cm] (float): "))
        self.shelf_width = float(input("Enter width of shelf in [cm] (float): "))
        self.shelf_depth = float(input("Enter depth of shelf in [cm] (float): "))
        print("=== Board dimensions === ")
        self.vertical_board_thickness = float(input("Enter thickness of vertical boards in [cm] (float): "))
        self.horizontal_board_thickness = float(input("Enter thickness of horizontal board in [cm] (float): "))
        self.shelf_board_depth = float(input("Enter the depth of the shelf boards in [cm] (float): "))
        print("=== Shelf boards ===")
        self.number_shelf_boards = int(input("Enter number of shelf boards (exclusive the board on top) (int): "))
        for i in range(self.number_shelf_boards):
            if (i == self.number_shelf_boards - 1):
                 self.clearances_above_shelf_boards.append(float(input(f"Enter distance between shelf board {i} and the top board in [cm] (float): ")))
            else:
                self.clearances_above_shelf_boards.append(float(input(f"Enter distance between shelf board {i} and {i+1} in [cm] (float): ")))
        self.height_first_board = float(input("Enter the distance from floor to upper edge of lowest shelf board in [cm] (float): "))
        print("=== Affordances ===")
        self.do_create_affordances = input("Create affordances? (y/n): ").lower().startswith('y')
        if self.do_create_affordances:
            self.create_affordances_for = [abs(int(x)) for x in input("Enter indices of shelf boards for which affordances should be generated, seperated by blankspace (and beginning at index 0): ").split()]
            if len(self.create_affordances_for) > self.number_shelf_boards or max(self.create_affordances_for) > self.number_shelf_boards:
                print("You entered an ambigous configuration of affordances, aborting...")
                sys.exit(1)
            self.do_pad_affordances = input("Add padding to affordances? (y/n): ").lower().startswith('y')
            if self.do_pad_affordances:
                self.affordance_padding['x'] = float(input("Padding applied on the left/right in [cm] (float): "))
                self.affordance_padding['y'] = float(input("Padding applied in the front/back in [cm] (float): "))
            self.affordance_max_z = float(input("Max height for all affordances in [cm] (float). Set to 0.0 to fill whole gap to next board: "))

    def _read_config(self):
        with open('shelf_config.yaml') as config_file:
            try:
                self.CONFIG = yaml.safe_load(config_file)

            except yaml.YAMLError as exc:
                print(exc)
                sys.exit(1)
        
    def _load_config(self):
        self.shelf_height = self.CONFIG['shelf_height']
        self.shelf_width = self.CONFIG['shelf_width']
        self.shelf_depth = self.CONFIG['shelf_depth']
        self.vertical_board_thickness = self.CONFIG['vertical_board_thickness']
        self.horizontal_board_thickness = self.CONFIG['horizontal_board_thickness']
        self.shelf_board_depth = self.CONFIG['shelf_board_depth']
        self.clearances_above_shelf_boards = self.CONFIG['clearances_above_shelf_boards']
        self.number_shelf_boards = len(self.clearances_above_shelf_boards)
        self.height_first_board = self.CONFIG['height_first_board']

        self.do_create_affordances = self.CONFIG['do_create_affordances']
        self.do_pad_affordances = self.CONFIG['do_pad_affordances']
        self.affordance_padding = self.CONFIG['affordance_padding']
        self.create_affordances_for = self.CONFIG['create_affordances_for']
        self.affordance_max_z = self.CONFIG['affordance_max_z']

    def _convert_to_meters(self):
        self.shelf_height /= 100.0
        self.shelf_width /= 100.0
        self.shelf_depth /= 100.0
        self.vertical_board_thickness /= 100.0
        self.horizontal_board_thickness /= 100.0
        self.shelf_board_depth /= 100.0
        self.clearances_above_shelf_boards = [x / 100.0 for x in self.clearances_above_shelf_boards]
        self.height_first_board /= 100.0
        self.affordance_padding['x'] /= 100.0
        self.affordance_padding['y'] /= 100.0
        self.affordance_max_z /= 100.0

    def _test_input(self):
        self.shelf_height = 50.0 #cm
        self.shelf_width = 30.0 #cm
        self.shelf_depth = 10.0 #cm
        self.vertical_board_thickness = 2.0 #cm
        self.horizontal_board_thickness = 2.0 #cm
        self.shelf_board_depth = 9.0 #cm
        self.clearances_above_shelf_boards = [20.0, 14.0] #cm
        self.height_first_board = 12.0 #cm
        self.number_shelf_boards = 2 #amount
        self.do_create_affordances = True
        self.create_affordances_for = [0, 1]
        self.affordance_max_z = 50

    def _generate(self):
        self.simple_geometry['primitives'].append(self._generate_vertical_board_primitive(True))
        self.simple_geometry['primitives'].append(self._generate_vertical_board_primitive(False))
        self.simple_geometry['primitives'].append(self._generate_back_primitive())
        for i in range(self.number_shelf_boards + 1):
            self.simple_geometry['primitives'].append(self._generate_horizontal_board_primitive(i))
        if (self.do_create_affordances):
            for board_index in self.create_affordances_for:
                self.affordances['storages'].append(self._generate_affordance(board_index))

        

    def _export_to_yaml(self):
        with open('regal-o-mat-output/simple_geometry.yaml', 'w') as outfile:
            yaml.dump(self.simple_geometry, outfile, default_flow_style=False)
        if self.do_create_affordances:
            with open('regal-o-mat-output/affordances.yaml', 'w') as outfile:
                yaml.dump(self.affordances, outfile, default_flow_style=False)

    def _generate_vertical_board_primitive(self, is_negative_x: bool) -> dict:
        return {
            'box' : {
                'pose' : {
                    'x' : -(self.shelf_width / 2 - self.vertical_board_thickness / 2) if is_negative_x else (self.shelf_width / 2 - self.vertical_board_thickness / 2),
                    'y' : 0.0,
                    'z' : self.shelf_height / 2
                },
                'size' : {
                    'x': self.vertical_board_thickness, 
                    'y': self.shelf_depth, 
                    'z': self.shelf_height
                }
            }
        }

    def _generate_back_primitive(self) -> dict:
        return {
            'box' : {
                'pose' : {
                    'x' : 0.0,
                    'y' : -self.shelf_depth / 2 + (self.shelf_depth - self.shelf_board_depth) / 2,
                    'z' : self.shelf_height / 2
                },
                'size' : {
                    'x' : self.shelf_width - 2 * self.vertical_board_thickness, 
                    'y' : self.shelf_depth - self.shelf_board_depth,
                    'z' : self.shelf_height
                }
            }
        }

    def _generate_horizontal_board_primitive(self, board_index: int) -> dict:
        return {
            'box' : {
                'pose' : {
                    'x' : 0.0,
                    'y' : (self.shelf_depth - self.shelf_board_depth) / 2,
                    'z' : self.height_first_board - self.horizontal_board_thickness / 2 if board_index == 0 else 
                    self.height_first_board 
                    + sum(self.clearances_above_shelf_boards[:board_index]) 
                    + (board_index - 1) * self.horizontal_board_thickness 
                    + self.horizontal_board_thickness / 2
                },
                'size' : {
                    'x': self.shelf_width - 2 * self.vertical_board_thickness, 
                    'y': self.shelf_board_depth, 
                    'z': self.horizontal_board_thickness
                }
            }
        }

    def _generate_affordance(self, board_index: int) -> dict:
        return {
            'name' : f"shelf_{board_index}",   
            'link_position' : 'bottom',
            'box' : {
                'pose' : {
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : self.height_first_board if board_index == 0 else 
                    self.height_first_board 
                    + sum(self.clearances_above_shelf_boards[:board_index]) 
                    + (board_index) * self.horizontal_board_thickness 
                },
                'size' : {
                    'x' : self.shelf_width - 2 * self.vertical_board_thickness - 2 * self.affordance_padding['x'],
                    'y' : self.shelf_board_depth - 2 * self.affordance_padding['y'],
                    'z' : self.clearances_above_shelf_boards[board_index] if self.affordance_max_z == 0.0 else min(self.affordance_max_z, self.clearances_above_shelf_boards[board_index])
                }
            }

        }
        
    
        
        


if __name__ == '__main__':
    shelf_gen = ShelfGenerator()
    shelf_gen.run()
    #shelf_gen.run_test()
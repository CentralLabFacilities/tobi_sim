#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import cv2
import math
import argparse
import os
import datetime

class XMLConstants:
    """
    Holds xml boilerplate needed for mujoco xml.
    """
    def __init__(self):
        self.SCENE_XML_HEADER = '<?xml version="1.0"?>\n\
                <mujoco xmlns:xacro="http://ros.org/wiki/xacro" model="Tiago Scene">\n\
                  \t<compiler angle="radian" autolimits="true" balanceinertia="true" />\n\
                  \t<option noslip_iterations="1" />\n\
                  \t<size njmax="500" nconmax="100" />\n\n'

        self.SCENE_INCLUDE_ROBOT = '\t<xacro:include filename="$(find tobi_sim)/scenes/makros/add_robot.xml.xacro" />\n\
                  \t<xacro:add_robot robot_name="tiago" robot_pos="0 0 0.11" robot_rot="0 0 0" />\n\n'

        self.SCENE_BASIC_SETUP = '\t<statistic center="0 0 0.3" extent="1.2" />\n\n\
                  \t<visual>\n\
                    \t<headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />\n\
                    \t<rgba haze="0.15 0.25 0.35 1" />\n\
                    \t<global azimuth="120" elevation="-20" />\n\
                  \t</visual>\n\n\
                  \t<asset>\n\
                    \t<texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512"\
                      height="3072" />\n\
                    \t<texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"\
                      rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />\n\
                    \t<material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"\
                      reflectance="0.2" />\n\n\
                    \t<texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3"\
                      rgb2=".2 .3 .4" />\n\
                    \t<material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2" />\n\
                  \t</asset>\n\n\
                  \t<worldbody>\n\
                    \t<light pos="0 0 1.5" dir="0 0 -1" directional="true" />\n\
                    \t<geom name="floor" size="0 0 0.05" type="plane" material="groundplane" />\n\
                  \t</worldbody>\n\n'

        self.SCENE_FOOTER = '</mujoco>'


class WallGenerator:
    def __init__(self, pixel_to_meter, map_image_path, save_file_path, do_add_robot, wall_thickness, wall_height):
        self.PIXEL_TO_METER = pixel_to_meter
        self.MAP_IMAGE_PATH = map_image_path
        self.SAVE_FILE_PATH = save_file_path
        self.DO_INCLUDE_ROBOT_IN_SCENE = do_add_robot
        self.MUJOCO_WALL_THICKNESS = wall_thickness / 2
        self.MUJOCO_WALL_HEIGHT = wall_height / 2

        self.NODE_COLOR_DEFAULT = 'orange'
        self.NODE_COLOR_SELECTED = 'red'
        self.EDGE_COLOR = 'cyan'

        self.XML_CONSTANTS = XMLConstants()

        if not self.MAP_IMAGE_PATH.endswith(".pgm"):
            raise ValueError(f"Invalid IMAGE_PATH: '{self.MAP_IMAGE_PATH}'. Must end with '.pgm'.")

        if not self.SAVE_FILE_PATH.endswith(".xml.xacro"):
            raise ValueError(f"Invalid SAVE_FILE_PATH: '{self.SAVE_FILE_PATH}'. Must end with '.xml.xacro'.")

        if not os.path.exists(self.MAP_IMAGE_PATH):
            raise FileNotFoundError(f"Input image not found: '{self.MAP_IMAGE_PATH}'.")

        self.IMAGE = cv2.imread(self.MAP_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)

        if self.IMAGE is None:
            raise FileNotFoundError(f"Image not found: {self.MAP_IMAGE_PATH}")

        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.IMAGE, cmap='gray', origin='upper', zorder=0)
        self.ax.set_title("Left click for adding new nodes. \nClick existing nodes to select them. \nWall gets created from selected node to new node. \nHit enter to save to xml. \nNodes can be dragged around but cant be deleted.")

        self.wall_connections = []
        self.wall_nodes = {}
        self.selected_wall_node_key = None
        self.dragged_wall_node_key = None
        self.drag_offset = np.array([0.0, 0.0])

        self.edges = []
        self.wall_previews = []

        self.segments = []
        self.xml_geoms = ['\t<worldbody>']

        self.cid_on_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_on_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_on_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_on_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        plt.show(block=True)

    def safe_close_figure(self):
        try:
            if hasattr(self.fig.canvas.manager, "window"):
                self.fig.canvas.manager.window.after(100, lambda: plt.close(self.fig))
            else:
                plt.close(self.fig)
        except Exception as e:
            print("Warning during figure close: ", e)

    def add_new_wall_node(self, plt_circle) -> int:
        new_wall_node_key = int(datetime.datetime.now().strftime('%Y%m%d%H%M%S%f'))
        self.wall_nodes[new_wall_node_key] = plt_circle
        if self.selected_wall_node_key is not None:
            self.wall_connections.append((self.selected_wall_node_key, new_wall_node_key))
        return new_wall_node_key

    def set_selected_wall_node_key(self, new_key):
        if self.selected_wall_node_key is not None:
            self.wall_nodes[self.selected_wall_node_key].set_edgecolor(self.NODE_COLOR_DEFAULT)
            if new_key == self.selected_wall_node_key:
                self.selected_wall_node_key = None
                return
        self.selected_wall_node_key = new_key
        self.wall_nodes[self.selected_wall_node_key].set_edgecolor(self.NODE_COLOR_SELECTED)

    def find_near_node(self, x, y, tolerance=12):
        for node_key, plt_circle in self.wall_nodes.items():
            cx, cy = plt_circle.center
            if np.hypot(cx - x, cy - y) <= tolerance:
                return node_key
        return None

    def update_edges(self):
        for edge in self.edges:
            edge.remove()
        self.edges.clear()
        for key_start_node, key_end_node in self.wall_connections:
            p1 = self.wall_nodes[key_start_node].center
            p2 = self.wall_nodes[key_end_node].center
            line = mlines.Line2D([p1[0], p2[0]], [p1[1], p2[1]], color=self.EDGE_COLOR, linewidth=2, zorder=4)
            self.ax.add_line(line)
            self.edges.append(line)

    def on_press(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return

        x, y = event.xdata, event.ydata
        is_ctrl_pressed = event.key == 'control'

        key_nearest_node = self.find_near_node(x, y)

        if is_ctrl_pressed and key_nearest_node is not None:
            self.set_selected_wall_node_key(key_nearest_node)
            self.fig.canvas.draw_idle()
            return

        if key_nearest_node is not None:
            self.dragged_wall_node_key = key_nearest_node
            self.drag_offset = np.array(self.wall_nodes[key_nearest_node].center) - np.array([x, y])
            self.set_selected_wall_node_key(key_nearest_node)
            self.fig.canvas.draw_idle()
            return

        plt_circle = patches.Circle(
            (x, y),
            radius=10,
            edgecolor=self.NODE_COLOR_DEFAULT,
            facecolor='none',
            lw=3,
            zorder=5
        )

        self.ax.add_patch(plt_circle)
        self.set_selected_wall_node_key(self.add_new_wall_node(plt_circle))

        self.update_edges()

        self.fig.canvas.draw_idle()

    def on_motion(self, event):
        if self.dragged_wall_node_key is None or event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata
        new_center = np.array([x, y]) + self.drag_offset
        self.wall_nodes[self.dragged_wall_node_key].center = new_center

        self.update_edges()
        self.fig.canvas.draw_idle()

    def on_release(self, event):
        self.dragged_wall_node_key = None

        self.update_edges()
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == 'enter':
            coords_m = {
                key: np.array(plt_circle.center) * self.PIXEL_TO_METER
                for key, plt_circle in self.wall_nodes.items()
            }

            coords_m = {k: np.array([v[0], -v[1]]) for k, v in coords_m.items()}

            sorted_points = [v for _, v in sorted(coords_m.items())[:2]]
            origin = sorted_points[0]

            coords_m = {k: v - origin for k, v in coords_m.items()}

            dx, dy = sorted_points[1] - sorted_points[0]
            theta = math.atan2(dy, dx)

            coords_m = {k: self.rotate_points(v, theta) for k, v in coords_m.items()}

            self.compute_geometry(coords_m)
            self.segments_to_mujoco_xml()
            self.save_to_file()

            self.safe_close_figure()
            print(f"Saved xml.xacro with wall geometries to '{self.SAVE_FILE_PATH}'")
            return

    def compute_geometry(self, coords_in_meter):
        for p1, p2 in self.wall_connections:
            x1, y1 = coords_in_meter[p1]
            x2, y2 = coords_in_meter[p2]
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)
            self.segments.append((x1, y1, x2, y2, length, angle))

    def segments_to_mujoco_xml(self):
        for idx, (x1, y1, x2, y2, length, angle) in enumerate(self.segments, 1):
            sx = length / 2
            sy = self.MUJOCO_WALL_THICKNESS
            sz = self.MUJOCO_WALL_HEIGHT
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2 + sy
            cz = self.MUJOCO_WALL_HEIGHT
            xml = (
                f'\t\t<geom name="wall{idx}" '
                f'pos="{cx:.3f} {cy:.3f} {cz:.3f}" '
                f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
                f'euler="0 0 {angle:.3f}" '
                f'type="box" rgba="0.5 0.5 0.5 1"/>'
            )
            self.xml_geoms.append(xml)
        self.xml_geoms.append('\t</worldbody>\n')

    def save_to_file(self):


        walls_xml_with_newlines = "\n".join(self.xml_geoms)
        with open(self.SAVE_FILE_PATH, "w") as f:
            f.write(self.XML_CONSTANTS.SCENE_XML_HEADER)
            if self.DO_INCLUDE_ROBOT_IN_SCENE:
                f.write(self.XML_CONSTANTS.SCENE_INCLUDE_ROBOT)
            f.write(self.XML_CONSTANTS.SCENE_BASIC_SETUP)
            f.write(walls_xml_with_newlines)
            f.write(self.XML_CONSTANTS.SCENE_FOOTER)

    @staticmethod
    def rotate_points(points, angle):
        c, s = np.cos(-angle), np.sin(-angle)
        rot_mat = np.array([[c, -s], [s, c]])
        return points @ rot_mat.T


if __name__ == '__main__':
    """
    Script: generate_scene.py
    Description:
        This script processes a map input image to generate a corresponding mujoco scene representation.
        It requires the paths to the input and output files, and takes optional
        parameters for scaling and wall dimensions and if the robot should be added to the scene or not.

    Usage:
        python generate_scene.py IMAGE_PATH SAVE_FILE_PATH [--pixel_to_meter VALUE] [--wall_thickness VALUE] [--wall_height VALUE] [--include_robot_in_scene]

    Example:
        python generate_scene.py input.pgm output.xml.xacro
    """
    DEFAULT_PIXEL_TO_METER = 0.05
    DEFAULT_WALL_THICKNESS = 0.1  # meters
    DEFAULT_WALL_HEIGHT = 0.5  # meters

    parser = argparse.ArgumentParser(
        description="Process a map (pgm file) to generate a corresponding mujoco scene."
    )
    parser.add_argument("MAP_IMAGE_PATH", help="Path to the input map image file (.pgm).")
    parser.add_argument("SAVE_FILE_PATH", help="Path to save the output scene file (.xml.xacro).")

    parser.add_argument("--pixel_to_meter",
                        type=float,
                        default=DEFAULT_PIXEL_TO_METER,
                        help="Conversion factor from pixel to meter (default 0.05)."
                        )
    parser.add_argument("--wall_thickness",
                        type=float,
                        default=DEFAULT_WALL_THICKNESS,
                        help="Wall thickness in meters in mujoco (default 0.1)."
                        )
    parser.add_argument("--wall_height",
                        type=float,
                        default=DEFAULT_WALL_HEIGHT,
                        help="Wall height in mujoco in meters (default 0.5).")
    parser.add_argument("--add_robot",
                        type=bool,
                        nargs="?",
                        default=False,
                        const=True,
                        help="Include robot in scene if set.")

    args = parser.parse_args()

    WallGenerator(args.pixel_to_meter, args.MAP_IMAGE_PATH, args.SAVE_FILE_PATH, args.add_robot, args.wall_thickness, args.wall_height)

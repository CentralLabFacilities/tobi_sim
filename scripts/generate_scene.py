#!/usr/bin/env python3
"""
Script: generate_scene.py
Description:
    This script processes an map input image to generate a corresponding mujoco scene representation.
    It requires the paths to the input and output files, and takes optional
    parameters for scaling and wall dimensions and if the robot should be added to the scene or not.

Usage:
    python generate_scene.py IMAGE_PATH SAVE_FILE_PATH [--pixel_to_meter VALUE] [--wall_thickness VALUE] [--wall_height VALUE] [--include_robot_in_scene]

Example:
    python generate_scene.py input.pgm output.xml.xacro
"""


import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import cv2
import math
import argparse
import os


DEFAULT_PIXEL_TO_METER = 0.05
DEFAULT_WALL_THICKNESS = 0.1  # meters
DEFAULT_WALL_HEIGHT = 0.5    # meters


SCENE_XML_HEADER = '<?xml version="1.0"?>\n\
<mujoco xmlns:xacro="http://ros.org/wiki/xacro" model="Tiago Scene">\n\
  \t<compiler angle="radian" autolimits="true" balanceinertia="true" />\n\
  \t<option noslip_iterations="1" />\n\
  \t<size njmax="500" nconmax="100" />\n\n'

SCENE_INCLUDE_ROBOT = '\t<xacro:include filename="$(find tobi_sim)/scenes/makros/add_robot.xml.xacro" />\n\
  \t<xacro:add_robot robot_name="tiago" robot_pos="0 0 0.11" robot_rot="0 0 0" />\n\n'

SCENE_BASIC_SETUP = '\t<statistic center="0 0 0.3" extent="1.2" />\n\n\
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

SCENE_FOOTER = '</mujoco>'

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
parser.add_argument("--add_robot_in_scene",
                    type=bool,
                    default=False,
                    help="Include robot in scene if set to true (default false).")

args = parser.parse_args()

PIXEL_TO_METER = args.pixel_to_meter
IMAGE_PATH = args.MAP_IMAGE_PATH
SAVE_FILE_PATH= args.SAVE_FILE_PATH
INCLUDE_ROBOT_IN_SCENE = args.add_robot_in_scene
WALL_THICKNESS = args.wall_thickness / 2 # Div by 2 because of 3D modelling convention
WALL_HEIGHT = args.wall_height / 2 # Div by 2 because of 3D modelling convention

if not IMAGE_PATH.endswith(".pgm"):
    raise ValueError(f"Invalid IMAGE_PATH: '{IMAGE_PATH}'. Must end with '.pgm'.")

if not SAVE_FILE_PATH.endswith(".xml.xacro"):
    raise ValueError(f"Invalid SAVE_FILE_PATH: '{SAVE_FILE_PATH}'. Must end with '.xml.xacro'.")

if not os.path.exists(IMAGE_PATH):
    raise FileNotFoundError(f"Input image not found: '{IMAGE_PATH}'.")


image = cv2.imread(IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
if image is None:
    raise FileNotFoundError(f"Image not found: {IMAGE_PATH}")

fig, ax = plt.subplots()
ax.imshow(image, cmap='gray', origin='upper', zorder=0)
ax.set_title("Click circles to connect; click an earlier one to close loop")

corners = cv2.goodFeaturesToTrack(
    image, maxCorners=27, qualityLevel=0.35, minDistance=10,
    blockSize=3, useHarrisDetector=False, k=0.04
)
corners = np.intp(corners)

circles = []
selected_circles = []
lines = []


def update_lines():
    for line in lines:
        line.remove()
    lines.clear()
    if len(selected_circles) >= 2:
        for i in range(len(selected_circles) - 1):
            p1 = selected_circles[i].center
            p2 = selected_circles[i + 1].center
            line = mlines.Line2D([p1[0], p2[0]], [p1[1], p2[1]],
                                 color='cyan', linewidth=2, zorder=4)
            ax.add_line(line)
            lines.append(line)


def safe_close_figure():
    try:
        if hasattr(fig.canvas.manager, "window"):
            fig.canvas.manager.window.after(100, lambda: plt.close(fig))
        else:
            plt.close(fig)
    except Exception as e:
        print("Warning during figure close:", e)


def compute_geometry(coords_m):
    """Compute lengths and angles (radians) between consecutive points."""
    segments = []
    for i in range(len(coords_m) - 1):
        x1, y1 = coords_m[i]
        x2, y2 = coords_m[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        segments.append((x1, y1, x2, y2, length, angle))
    if len(coords_m) > 2:
        x1, y1 = coords_m[-1]
        x2, y2 = coords_m[0]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        segments.append((x1, y1, x2, y2, length, angle))
    return segments


def segments_to_mujoco_xml(segments):
    """Convert line segments into MuJoCo <geom> XML strings."""
    xml_geoms = ['\t<worldbody>']
    for i, (x1, y1, x2, y2, length, angle) in enumerate(segments, 1):
        sy = WALL_THICKNESS
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2 + sy
        cz = WALL_HEIGHT
        sx = length / 2
        sz = WALL_HEIGHT
        xml = (
            f'\t\t<geom name="wall{i}" '
            f'pos="{cx:.3f} {cy:.3f} {cz:.3f}" '
            f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
            f'euler="0 0 {angle:.3f}" '
            f'type="box" rgba="0.5 0.5 0.5 1"/>'
        )
        xml_geoms.append(xml)
    xml_geoms.append('\t</worldbody>\n')
    return xml_geoms

def save_to_file(walls_xml):
    walls_xml_with_newlines = "\n".join(walls_xml)
    with open(SAVE_FILE_PATH, "w") as f:
        f.write(SCENE_XML_HEADER)
        if INCLUDE_ROBOT_IN_SCENE:
            f.write(SCENE_INCLUDE_ROBOT)
        f.write(SCENE_BASIC_SETUP)
        f.write(walls_xml_with_newlines)
        f.write(SCENE_FOOTER)

        


def rotate_points(points, angle):
    """Rotate all 2D points by -angle radians."""
    c, s = np.cos(-angle), np.sin(-angle)
    rot_mat = np.array([[c, -s], [s, c]])
    return points @ rot_mat.T


def on_pick(event):
    artist = event.artist
    if not isinstance(artist, patches.Circle):
        return

    global selected_circles

    if artist in selected_circles:
        idx = selected_circles.index(artist)
        if idx != len(selected_circles) - 1 and len(selected_circles) > 2:
            p1 = selected_circles[-1].center
            p2 = selected_circles[0].center
            line = mlines.Line2D([p1[0], p2[0]], [p1[1], p2[1]],
                                 color='cyan', linewidth=2, zorder=4)
            ax.add_line(line)
            lines.append(line)
            fig.canvas.draw_idle()

            coords_px = np.array([c.center for c in selected_circles])
            coords_m = coords_px * PIXEL_TO_METER

            coords_m[:, 1] = -coords_m[:, 1]

            origin = coords_m[0]
            coords_m -= origin

            dx, dy = coords_m[1] - coords_m[0]
            theta = math.atan2(dy, dx)
            coords_m_rot = rotate_points(coords_m, theta)

            segments = compute_geometry(coords_m_rot)
            xml_geoms = segments_to_mujoco_xml(segments)

            save_to_file(xml_geoms)

            safe_close_figure()
            print(f"Saved xml.xacro with wall geometries to '{SAVE_FILE_PATH}'")
            return
        else:
            return

    selected_circles.append(artist)
    artist.set_edgecolor('red')
    artist.set_linewidth(3)
    update_lines()
    fig.canvas.draw_idle()


for (x, y) in corners.reshape(-1, 2):
    circ = patches.Circle((x, y), radius=10, edgecolor='yellow',
                          facecolor='none', lw=2, picker=True, zorder=5)
    ax.add_patch(circ)
    circles.append(circ)

fig.canvas.mpl_connect('pick_event', on_pick)
plt.show(block=True)
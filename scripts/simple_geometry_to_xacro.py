#!/usr/bin/env python3
"""
simple_geometry_to_xacro.py

Reads given simple_geometry.yaml file (-> ecwm) and generates a MuJoCo-compatible
Xacro file (<mujoco> root) that defines a <xacro:macro>. The macro produces a standalone
<worldbody> with a <body> containing <geom> elements.

Usage:
    python3 simple_geometry_to_xacro.py input.yaml output.xml.xacro

Optional rgba values:
    python3 simple_geometry_to_xacro.py input.yaml output.xml.xacro '0.8 0.5 0.3 1'

The macro name will automatically be derived from the output file name:
    e.g., output="geometry_macro.xml.xacro" -> macro name = "add_geometry_macro"
"""

import sys
import yaml
import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path


def prettify_xml(elem):
    """Returns a formatted XML string."""
    rough = ET.tostring(elem, "utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ")


def make_geom_element(prim_type, pose, size, idx, rgba, base_name="${geom_name}"):
    """
    Create an XML <geom> element for the given primitive.
    The geom name becomes "${geom_name}_{type}_{idx}".
    """
    geom_name = f"{base_name}_{prim_type}_{idx}"
    geom = ET.Element("geom", attrib={"name": geom_name, "type": prim_type})

    pos_x = float(pose.get("x", 0))
    pos_y = float(pose.get("y", 0))
    pos_z = float(pose.get("z", 0))
    geom.set("pos", f"{pos_x} {pos_y} {pos_z}")

    if prim_type == "box":
        size_x = float(size.get("x", 0)) / 2.0
        size_y = float(size.get("y", 0)) / 2.0
        size_z = float(size.get("z", 0)) / 2.0
        geom.set("size", f"{size_x} {size_y} {size_z}")
    elif prim_type == "sphere":
        radius = size.get("r", size.get("radius", None))
        if radius is None:
            raise ValueError("Sphere needs 'r' or 'radius'")
        geom.set("size", str(float(radius)))
    elif prim_type == "cylinder":
        radius = float(size.get("radius", size.get("r", 0)))
        height = float(size.get("height", size.get("z", 0))) / 2.0
        geom.set("size", f"{radius} {height}")
    else:
        raise ValueError(f"Unsupported primitive type: {prim_type}")

    geom.set("rgba", rgba)
    geom.set("density", "1000")
    return geom


def yaml_to_xacro_worldbody(yaml_path, macro_name, rgba="0.5 0.5 0.5 1"):
    """Convert YAML primitives into a <mujoco> Xacro macro."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    mujoco_root = ET.Element(
        "mujoco",
        attrib={"xmlns:xacro": "http://ros.org/wiki/xacro"},
    )

    macro = ET.SubElement(
        mujoco_root,
        "xacro:macro",
        attrib={"name": macro_name, "params": "geom_name"},
    )

    worldbody = ET.SubElement(macro, "worldbody")

    body = ET.SubElement(worldbody, "body", attrib={"name": "${geom_name}_body"})

    primitives = data.get("primitives", [])
    for idx, item in enumerate(primitives):
        if not isinstance(item, dict) or len(item) == 0:
            continue

        prim_type = next(iter(item.keys()))
        prim_spec = item[prim_type] or {}
        pose = prim_spec.get("pose", {})
        size = prim_spec.get("size", {})

        try:
            geom = make_geom_element(prim_type, pose, size, idx, rgba)
        except Exception as e:
            print(f"Warning: skipping {prim_type}_{idx} due to {e}", file=sys.stderr)
            continue

        body.append(geom)

    return mujoco_root


def derive_macro_name(output_path: str) -> str:
    """Derive macro name as 'add_' + basename of output file without .xml.xacro"""
    filename = Path(output_path).name
    if filename.endswith(".xml.xacro"):
        filename = filename.removesuffix(".xml.xacro")
    return f"add_{filename}"


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 simple_geometry_to_xacro.py input.yaml output.xml.xacro")
        sys.exit(1)

    yaml_path = sys.argv[1]
    out_path = sys.argv[2]

    macro_name = derive_macro_name(out_path)
    if len(sys.argv) >= 4:
        rgba = sys.argv[3]
        mujoco_root = yaml_to_xacro_worldbody(yaml_path, macro_name, rgba)
    else:
        mujoco_root = yaml_to_xacro_worldbody(yaml_path, macro_name)
    xml_str = prettify_xml(mujoco_root)

    with open(out_path, "w") as f:
        f.write(xml_str)

    print(f"Wrote Xacro macro '{macro_name}' to: {out_path}")


if __name__ == "__main__":
    main()
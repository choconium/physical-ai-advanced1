#!/usr/bin/env python3
"""Rewrite virtual test glyphs before a NEW trial. Never used during wiping."""
import xml.etree.ElementTree as ET
from ink_model import BoardConfig, initial_pixels, build_sdf
from gazebo_scene import GazeboScene

scene = GazeboScene()
root = ET.fromstring(build_sdf(BoardConfig(), initial_pixels()))
for model in root.findall('./world/model'):
    name = model.attrib['name']
    if not name.startswith('ink_'):
        continue
    try:
        scene.erase(name)
    except RuntimeError:
        pass  # previously erased cells are absent by design
    scene.create_xml(ET.tostring(model, encoding='unicode'))
print('Rewrote27 virtual glyph cells for a new trial')

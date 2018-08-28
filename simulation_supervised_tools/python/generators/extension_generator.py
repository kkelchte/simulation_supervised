#!/usr/bin/python
import xml.etree.ElementTree as ET
import numpy as np
import os,sys, time
import argparse

import yaml

import matplotlib.pyplot as plt

template_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/extensions/templates/'

def generate_panel(height=1,
                    width=0.7,
                    z_location=1,
                    offset=0,
                    texture= 'Gazebo/Grey',
                    wall=""):
  """
  Args:
  height of the panel
  width of the panel
  z_location above the ground
  offset is the relative position in the corridor segment
  specific texture that has to be found in simsup_demo/extensions/textures
  wall (left, right, front or back) ==> if '' pick random (left,right)
  Returns model placed in centered segment.
  """
  if not wall: wall=np.random.choice(["right","left"])
  # Get template
  panel_tree = ET.parse(template_location+'panel.xml')
  panel = panel_tree.getroot().find('world').find('model')
  # change shape of panel according to heigth and width
  for child in iter(['collision', 'visual']):
    size_el=panel.find('link').find(child).find('geometry').find('box').find('size')
    size=[float(v) for v in size_el.text.split(' ')]
    size_el.text=str(size[0])+" "+str(width)+" "+str(height)

  # change position of panel according to wall, z_location and offset
  pose_el=panel.find('pose')
  offset = np.sign(offset)*(np.abs(offset)-width/2)
  position={"left":[-1, offset],
            "right":[1, offset],
            "front":[offset, 1],
            "back":[offset, -1],}
  pose_6d = [float(v) for v in pose_el.text.split(' ')]
  pose_el.text=str(position[wall][0])+" "+str(position[wall][1])+" "+str(z_location+height/2.)+" "+str(pose_6d[3])+" "+str(pose_6d[4])+" "+str(pose_6d[5])
  # correct texture
  material=panel.find('link').find('visual').find('material').find('script').find('name')
  material.text=texture
  return panel

def generate_passway(height= 1,
                    width= 0.7,
                    z_location= 1,
                    name= 'arc',
                    texture= 'Gazebo/Grey',
                    tile_type=-1):
  """
  """

  return ET.Element('None')

def generate_obstacle(name='human',
                      scale=0.7,
                      x_location=1,
                      y_location=1,
                      tile_type=-1):
  """
  """

  return ET.Element('None')

def generate_ceiling(length=1,
                    height= 1,
                    width= 0.7,
                    offset= 1,
                    orientation= 'parallel',
                    texture= 'Gazebo/Grey',
                    tile_type=-1):
  """
  """

  return ET.Element('None')

def generate_blocked_hole(height= 1,
                          width= 0.7,
                          z_location= 1,
                          texture= 'Gazebo/Grey',
                          tile_type=-1):
  """
  """

  return ET.Element('None')

#!/usr/bin/python
import xml.etree.ElementTree as ET
import numpy as np
import os,sys, time
import argparse

import yaml

import matplotlib.pyplot as plt

def generate_blocked_hole(height= 1,
                          width= 0.7,
                          z_location= 1,
                          texture= 'Gazebo/Grey',
                          tile_type=-1):
  """
  """

  return ET.Element('None')


def generate_panel(height= 1,
                    width= 0.7,
                    z_location= 1,
                    texture= 'Gazebo/Grey',
                    tile_type=-1):
  """
  """

  return ET.Element('None')

def generate_passway(height= 1,
                    width= 0.7,
                    z_location= 1,
                    name= 'arc',
                    texture= 'Gazebo/Grey',
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

def generate_obstacle(name='human',
                      scale=0.7,
                      x_location=1,
                      y_location=1,
                      tile_type=-1):
  """
  """

  return ET.Element('None')

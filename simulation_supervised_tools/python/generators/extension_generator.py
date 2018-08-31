#!/usr/bin/python
import xml.etree.ElementTree as ET
import numpy as np
import os,sys, time
import argparse

import yaml

import matplotlib.pyplot as plt

template_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/extensions/templates/'


prefab_textures=['Gazebo/Grey','Gazebo/Blue','Gazebo/Red','Gazebo/Green','Gazebo/White','Gazebo/Black']
prefab_obstacles=['person_standing','bookshelf']

def generate_panel(dummy='', # used to set a dummy argument in order to avoid empty argument dictionaries
  height=None,
  width=None,
  thickness=None,
  z_location=None,
  offset=None,
  texture=None,
  wall=None,
  verbose=False):
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
  # fill in randomly all that is not specified
  if height==None: height = np.random.uniform(0.1,2)
  if width==None: width = np.random.uniform(0.1,2)
  if thickness==None: thickness = np.random.uniform(0.001,0.3)
  if z_location==None: z_location = np.random.uniform(0,1) #ALLWAYS BETWEEN 0 and 1
  if offset==None: offset = np.random.uniform(-0.5,0.5)
  if texture==None: texture = np.random.choice(prefab_textures)
  if wall==None: texture = np.random.choice(["right","left"])
  
  if verbose: print("[generate_panel]: height {0}, width {1}, thicknes {2}, z_location {3}, offset {4}, texture {5}, wall {6}".format(height,width,thickness,z_location,offset,texture,wall))
  # Get template
  panel_tree = ET.parse(template_location+'panel.xml')
  panel = panel_tree.getroot().find('world').find('model')
  # change shape of panel according to heigth and width
  for child in iter(['collision', 'visual']):
    size_el=panel.find('link').find(child).find('geometry').find('box').find('size')
    # size=[float(v) for v in size_el.text.split(' ')]
    # thickness is multiplied as the center of the panel remains in the wall
    # this ensures the offset by multiplying with width/2 remains correctly
    size_el.text=str(2*thickness)+" "+str(width)+" "+str(height)

  # change position of panel according to wall, z_location and offset
  pose_el=panel.find('pose')
  offset = np.sign(offset)*(np.abs(offset)-width/2)
  position={"left":[-1, offset],
            "right":[1, offset],
            "front":[offset, 1],
            "back":[offset, -1]}
  orientation={"left":0,
              "right":0,
              "front":1.57,
              "back":1.57}
  pose_6d = [float(v) for v in pose_el.text.split(' ')]
  pose_el.text=str(position[wall][0])+" "+str(position[wall][1])+" "+str(z_location*(2-height)+height/2.)+" "+str(pose_6d[3])+" "+str(pose_6d[4])+" "+str(orientation[wall])
  # adjust texture
  material=panel.find('link').find('visual').find('material').find('script').find('name')
  material.text=texture
  return panel

def generate_passway(dummy='',
  name=None,
  model_dir='',
  offset=None,
  texture=None,
  verbose=False):
  """Get a passway from simsup_demo/models and place it in the middle of the tile,
  adjust texture and offset.
  """
  if name == None: name=np.random.choice(['arc','doorway'])
  if offset == None: offset=np.random.uniform(-0.8,0.8)
  if texture==None: texture = np.random.choice(prefab_textures)
  if not model_dir: model_dir=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/models'
  if not os.path.isfile(model_dir+'/'+name+'/model.sdf'):
    print("[extension_generator]: failed to load model {0}, not in {1}".format(name, model_dir))
    return -1
  
  if verbose: print("[generate_passway]: name {0}, offset {1}, texture {2}".format(name, offset, texture))
  
  # load element
  passway_tree = ET.parse(model_dir+'/'+name+'/model.sdf')
  passway = passway_tree.getroot().find('model')

  # adjust position
  pose_6d = [float(v) for v in passway.find('pose').text.split(' ')]
  pose_6d[1] = offset
  passway.find('pose').text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])

  # adjust texture
  material=passway.find('link').find('visual').find('material').find('script').find('name')
  material.text=texture
  return passway

def generate_obstacle(dummy='',
  name=None,
  model_dir='',
  side_offset=None,
  offset=None,
  wall=None,
  verbose=False):
  """ Get an element from simsup_demo/models and place it on the side of the tile.
  """
  if name == None: name=np.random.choice(prefab_obstacles)
  if side_offset == None: offset=np.random.uniform(0.7,0.9)
  if offset == None: offset=np.random.uniform(-0.5,0.5)
  if wall == None: wall = np.random.choice(["right","left"])
  
  if not model_dir: model_dir=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/models'
  if not os.path.isfile(model_dir+'/'+name+'/model.sdf'):
    print("[extension_generator]: failed to load model {0}, not in {1}".format(name, model_dir))
    return -1

  if verbose: print("[generate_obstacle]: name {0}, offset {1}, wall {2}".format(name, offset, wall))

  # load element
  obstacle_tree = ET.parse(model_dir+'/'+name+'/model.sdf')
  obstacle = obstacle_tree.getroot().find('model')

  # adjust position
  position={"left":[-side_offset, offset],
            "right":[side_offset, offset],
            "front":[offset, side_offset],
            "back":[offset, -side_offset]}
  orientation={"left":0,
              "right":0,
              "front":1.57,
              "back":1.57}
  pose_6d = [float(v) for v in obstacle.find('pose').text.split(' ')]
  obstacle.find('pose').text = str(position[wall][0])+" "+str(position[wall][1])+" "+str(pose_6d[2])+" "+str(pose_6d[3])+" "+str(pose_6d[4])+" "+str(orientation[wall])

  return obstacle

def generate_ceiling(dummy='',
  name=None,
  model_dir='',
  tile_type=1,
  length = 2,
  texture=None,
  verbose=False):
  """Load model with name 'name_tile_type' {1,2 or 3} from model_dir.
  Adjust the texture accordingly.
  """
  if tile_type in [0,4]: tile_type=1 #the start and end tile is the same as straight
  if name == None: name=np.random.choice(['pipes','ceiling'])
  if not model_dir: model_dir=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/models'
  if not os.path.isfile(model_dir+'/'+name+'_'+str(tile_type)+'/model.sdf'):
    print("[extension_generator]: failed to load model {0}, not in {1}".format(name, model_dir))
    return -1  
  if verbose: print("[generate_ceiling]: name {0}, tile_type {1}, texture {2}".format(name, tile_type, texture))
  
  # load element
  ceiling_tree = ET.parse(model_dir+'/'+name+'_'+str(tile_type)+'/model.sdf')
  ceiling_models = ceiling_tree.getroot().findall('model')

  # adjust length of elements
  if name in ['pipes']:
    for ceiling in ceiling_models:
      for child in iter(['collision', 'visual']):
        length_el=ceiling.find('link').find(child).find('geometry').find('cylinder').find('length')
        length_el.text = str(length)
  elif name == 'ceiling':
    # add straight segments before the centered segment
    extra_length = (length-1.)/2. #0.5 in case of length 2
    for i in np.arange(0, extra_length, 1):
      # add straight segments on correct location according to tile type
      straight_ceiling_tree = ET.parse(model_dir+'/'+name+'_'+str(1)+'/model.sdf')
      straight_ceiling_models = straight_ceiling_tree.getroot().findall('model')
      for ceiling in straight_ceiling_models:
        pose_6d=[float(v) for v in ceiling.find('pose').text.split(' ')]
        pose_6d[1] = -1*(i+1)
        ceiling.find('pose').text=str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
        ceiling_models.append(ceiling)
    
    # add straight segments before the centered segment
    extra_length = (length-1.)/2. #0.5 in case of length 2
    for i in np.arange(0, extra_length, 1):
      # add straight segments on correct location according to tile type
      straight_ceiling_tree = ET.parse(model_dir+'/'+name+'_'+str(1)+'/model.sdf')
      straight_ceiling_models = straight_ceiling_tree.getroot().findall('model')
      for ceiling in straight_ceiling_models:
        pose_6d=[float(v) for v in ceiling.find('pose').text.split(' ')]
        if tile_type == 1: 
          pose_6d[1] = (i+1)
        elif tile_type == 2: 
          pose_6d[0] = (i+1)
          pose_6d[5] = 1.57
        elif tile_type == 3: 
          pose_6d[0] = -(i+1)
          pose_6d[5] = 1.57
        ceiling.find('pose').text=str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
        ceiling_models.append(ceiling)
    
  # adjust texture
  if texture != None:
    for ceiling in ceiling_models:
      material=ceiling.find('link').find('visual').find('material').find('script').find('name')
      material.text=texture
  return ceiling_models

def generate_blocked_hole(wall,
  width,
  height,
  dummy='',
  name='blocked_hole_segment.world',
  world_dir='',
  texture=None,
  corridor_type='normal',
  verbose=False):
  """
  wall: wall has to specified
  name: name of segment world file from which block_hole is extracted
  modeldir: world directory
  texture: in case of None take default defined in blocked_hole_segment
  verbose: talk more
  """
  if len(world_dir)==0: world_dir=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds'
  if not os.path.isfile(world_dir+'/'+name):
    print("[generate_blocked_hole]: failed to load model {0}, not in {1}".format(name, world_dir))
    return -1  
  if verbose: print("[generate_blocked_hole]: name {0}, wall {1}, texture {2}".format(name, wall, texture))
  
  # load model from blocked world segment
  tree = ET.parse(world_dir+'/'+name)
  root = tree.getroot()
  world = root.find('world')

  elements=[ m for m in world.findall('model') if m.attrib['name'].startswith('wall_'+wall) ]
  
  # incase the corridor_type is not normal but empty, make the wall much larger
  if corridor_type == 'empty':
    height = 10


  # adjust scale according to width and height
  # wall are made in blocked_hole_segment of width 2 and height 2.5
  # the inside wall are kept the same shape as they represent the door
  # the front walls are adjusted to the width and height
  for m in elements:
    if 'front_left' in m.attrib['name'] or 'front_right' in m.attrib['name']:
      # adjust width and height of left and right front wall
      for e in ['collision','visual']:
        size_element = m.find('link').find(e).find('geometry').find('box').find('size')
        size = [float(v) for v in size_element.text.split(' ')]
        # side with corresponds on the tile-width that influences the width of the wall
        # the width of the tile influences both the length of the wall as the position from the center
        # only the length should be changed (side_width) to a large value in case of empty corridor
        side_width = width if corridor_type == 'normal' else 50
        size[0] = (side_width-1)/2.
        size[2] = height
        size_element.text=str(size[0])+' '+str(size[1])+' '+str(size[2])
      # adjust pose according to width
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      
      if wall == 'right':
        pose_6d[0] = width/2.
        # half of the width of the front panel plus 0.5 for a door of width 1
        pose_6d[1] = ((side_width-1)/2./2.+0.5)
        pose_6d[1] = pose_6d[1] if 'front_left' in m.attrib['name'] else -pose_6d[1] #change in opposite direction on the right side
      elif wall == 'left':
        pose_6d[0] = -width/2.
        pose_6d[1] = ((side_width-1)/2./2.+0.5)
        pose_6d[1] = pose_6d[1] if 'front_left' in m.attrib['name'] else -pose_6d[1]
      elif wall == 'front':
        pose_6d[1] = width/2.
        pose_6d[0] = ((side_width-1)/2./2.+0.5)
        pose_6d[0] = pose_6d[0] if 'front_left' in m.attrib['name'] else -pose_6d[0] 
      elif wall == 'back':
        pose_6d[1] = -width/2.
        pose_6d[0] = ((side_width-1)/2./2.+0.5)
        pose_6d[0] = pose_6d[0] if 'front_left' in m.attrib['name'] else -pose_6d[0] 

      pose_6d[2] = height/2.
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
    if 'front_up' in m.attrib['name']:
      # adjust height of up pannel
      for e in ['collision','visual']:
        size_element = m.find('link').find(e).find('geometry').find('box').find('size')
        size = [float(v) for v in size_element.text.split(' ')]
        size[2] = height-2
        size_element.text=str(size[0])+' '+str(size[1])+' '+str(size[2])
      # adjust pose of up panel according to width and height
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      # in case of right wall
      
      if wall == 'right': pose_6d[0] = width/2.
      elif wall == 'left': pose_6d[0] = -width/2.
      elif wall == 'front': pose_6d[1] = width/2.
      elif wall == 'back': pose_6d[1] = -width/2.
      
      
      
      # half of the width of the front panel plus 0.5 for a door of width 1
      pose_6d[2] = (height-2)/2.+2
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
    if 'inside' in m.attrib['name']:
      # move the door a bit back according to the width of the corridor
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      
      # in case of right wall
      if wall == 'right': pose_6d[0] = width/2.+0.25 if not 'back' in m.attrib['name'] else width/2.+0.5
      elif wall == 'left': pose_6d[0] = -(width/2.+0.25) if not 'back' in m.attrib['name'] else -(width/2.+0.5)
      elif wall == 'front': pose_6d[1] = width/2.+0.25 if not 'back' in m.attrib['name'] else width/2.+0.5
      elif wall == 'back': pose_6d[1] = -(width/2.+0.25) if not 'back' in m.attrib['name'] else -(width/2.+0.5)
      
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])

    # change texture
    if texture != None:
      material_element= m.find('link').find('visual').find('material').find('script').find('name')
      material_element.text = texture

  return elements

def generate_floor(width,
  dummy='', # used to set a dummy argument in order to avoid empty argument dictionaries
  name=None,
  texture=None,
  corridor_type='normal',
  tile_type=1,
  verbose=False):
  """
  Args:
  width of the tile
  specific texture that has to be found in simsup_demo/extensions/textures
  Returns model placed in centered segment.
  """
  # fill in randomly all that is not specified
  if texture==None: texture = np.random.choice(prefab_textures)
  if name==None: name = np.random.choice(['floor'])
  
  if verbose: print("[generate_floor]: texture {0}".format(texture))
  # Get template
  floor_tree = ET.parse(template_location+name+'.xml')
  floor = floor_tree.getroot().find('world').find('model')
  # change shape of floor according to heigth and width
  for child in iter(['collision', 'visual']):
    size_el=floor.find('link').find(child).find('geometry').find('box').find('size')
    if corridor_type == 'empty' and tile_type != 4:
      size_el.text=str(20*width)+" "+str(20*width)+" "+size_el.text.split(' ')[-1]
    else:
      size_el.text=str(2*width)+" "+str(2*width)+" "+size_el.text.split(' ')[-1]
  # adjust texture
  material=floor.find('link').find('visual').find('material').find('script').find('name')
  material.text=texture
  return floor


#!/usr/bin/python
import xml.etree.ElementTree as ET
import numpy as np
import os,sys, time

from extension_generator import *

import matplotlib.pyplot as plt

###### Util Functions
def pretty_append(world, element):
  """
  Append a model to the world but ensure the indentation is done correctly.
  """
  normal_tail=world[-2].tail # get normal indentation-tail ex '\n    '
  last_tail=world[-1].tail # get indentation-tail for last element ex '\n  '
  world[-1].tail = normal_tail # give current last element the correct normal indentation-tail
  world.append(element) # add new last element
  world[-1].tail = last_tail # give new last element correct last indentation-tail

def visualize(sequence, save_location=""):
  """Debugging function displays sequence
  """
  # translate to instructions to be printed
  translate={0:'start',
            1:'straight',
            2:'right',
            3:'left',
            4:'stop'}
  print "translate to directions: "
  msg=''  
  for t in sequence:
    msg=msg+' '+translate[t]
  print msg

  # parse locations with tiles
  visited_tiles = from_sequence_to_tiles(sequence)
  
  # visualize visited locations with matplotlib
  c='#94a3ba'
  for i,t in enumerate(visited_tiles[1:]):
    plt.plot([visited_tiles[i].x,visited_tiles[i+1].x],
            [visited_tiles[i].y,visited_tiles[i+1].y],
            color=c, marker='',linestyle='solid',linewidth=100./len(sequence))
  plt.gca().set_aspect('equal', adjustable='box')

  plt.ylim(ymin=-len(sequence),
          ymax=len(sequence))
  plt.xlim(xmin=-len(sequence),
          xmax=len(sequence))

  if save_location:
    plt.savefig(save_location, bbox_inches='tight')
  else:
    plt.show()


####### Tile classe
class Tile(object):
  """A Tile has a position and an entering orientation"""
  def __init__(self, x, y, orientation, tile_type):
    walls= {0: ['top','bottom','back', 'left', 'right'],
        1: ['top','bottom','left', 'right'],
        2: ['top','bottom','front', 'left'],
        3: ['top','bottom','front', 'right'],
        4: ['top','bottom', 'left', 'right']}
    self.generator_dic={
      'panel': generate_panel,
      'passway': generate_passway,
      'obstacle': generate_obstacle,
      'ceiling': generate_ceiling,
      'blocked_hole': generate_blocked_hole,
    }
    self.x=x
    self.y=y
    self.o=orientation
    self.tile_type = tile_type
    self.extensions = []
    self.walls = walls[self.tile_type]

  def update_orientation(self):
    """turn clockwise or counterclockwise or stay"""
    orientation_list=["+y","+x","-y","-x"]
    if self.tile_type==3:
      # turn left ~ counter clockwise ~ down in orientation_list
      index = orientation_list.index(self.o)
      new_index = (index-1)%4
      return orientation_list[new_index]
    elif self.tile_type==2:
      # turn right ~ clockwise ~ up in orientation_list
      index = orientation_list.index(self.o)
      new_index = (index+1)%4
      return orientation_list[new_index]
    else: #go straight in case of tile type 0, 1 or 4
      return self.o

  def update_position(self,orientation):
    """change position in direction of new orientation"""
    relative_position={"+y":(0,1),
                      "+x":(1,0),
                      "-y":(0,-1),
                      "-x":(-1,0)}
    return (self.x+relative_position[orientation][0], self.y+relative_position[orientation][1])

  def leave(self, new_tile_type):
    """When you leave a tile you enter a new one with a new orientation"""
    new_orientation=self.update_orientation()
    new_x,new_y=self.update_position(new_orientation)
    return Tile(new_x,new_y,new_orientation,new_tile_type)

  def get_wall_models(self, width, height, texture, visual=True, verbose=False):
    """Parses the wall models and position them accordingly.
    """
    # 1. get wall models for front, back, left, right wall depending on which type of tile
    worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
    template_world='segment.world'
    tree = ET.parse(worlds_location+template_world)
    root = tree.getroot()
    world = root.find('world')

    wall_models=[ m for m in world.findall('model') if m.attrib['name'][5:] in self.walls]
    
    # 2. change height, width and texture accordingly
    for m in wall_models:
      # adjust the pose of the segment according to the width and the height
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      pose_6d[0]*=width/2.
      pose_6d[1]*=width/2.
      # in case the height changes the side walls should be lowered half the height and the top should be lowered a full height
      if m.attrib['name'] in ['wall_front','wall_back', 'wall_left', 'wall_right']:  pose_6d[2]=height/2.
      if m.attrib['name'] == 'wall_top':  pose_6d[2]=height
      
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
      # scale the wall according to the width and the height
      link_element=m.find('link')
      for e in ['collision','visual']:
        element=link_element.find(e)
        size_element = element.find('geometry').find('box').find('size')
        size = [float(v) for v in size_element.text.split(' ')]
        size[0] = width
        size[2] = height if not m.attrib['name'] in ['wall_top','wall_bottom'] else width
        size_element.text=str(size[0])+' '+str(size[1])+' '+str(size[2])

      # change texture
      material_element=link_element.find('visual').find('material').find('script').find('name')
      material_element.text = texture

      # in case visual = False the walls should be invisible
      if not visual:  link_element.remove(link_element.find('visual'))

    # 3. adjust pose of wall models
    rotation_matrix={'+y':np.array([[1,0],[0,1]]),
          '-y':np.array([[-1,0],[0,-1]]), # turn over 180 degr
          '-x':np.array([[0,-1],[1,0]]), #turn +90degr around z (left)
          '+x':np.array([[0,1],[-1,0]])} #turn -90degr around z (right)
    thetas={'+y':0,
            '-y':3.14,
            '-x':1.57,
            '+x':-1.57}
    for m in wall_models:
      # step 1 rotate to correct orientation
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      position=np.array([pose_6d[0], pose_6d[1]])
      position=np.matmul(rotation_matrix[self.o], position)
      pose_6d[5]+=thetas[self.o]
      pose_6d[0]=position[0]
      pose_6d[1]=position[1]
      # step 2 translate
      pose_6d[0]+=width*self.x
      pose_6d[1]+=width*self.y
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
    
    return wall_models

  def get_lights(self, lights, width, height, verbose=False):
    """Lights are parsed from simsup_demo/lights/.
    """
    # add lights accordingly
    light_models=[]
    if not isinstance(lights,list):
      lights=[lights]
    for light in lights:
      lights_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/lights/'
      light_tree = ET.parse(lights_location+light)
      lightelements = light_tree.getroot().find('world').findall('light')
      for lightelement in lightelements:
        if lightelement.find('pose') is not None:
          pose_element=lightelement.find('pose')
          pose_6d=[float(v) for v in pose_element.text.split(' ')]
          pose_6d[0]+=width*self.x
          pose_6d[1]+=width*self.y
          pose_6d[2]=height-0.01
          pose_element.text = str(pose_6d[0])+" "+str(pose_6d[1])+" "+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])
        light_models.append(lightelement)
    return light_models

  def get_extensions(self, width, height, verbose=False):
    """ Go over list of self.extensions.
    Distribute the extensions over the different walls avoiding overlap.
    Get elements of the models with the extension_generator.
    return list of elements required for the extension.
    """
    # add extensions
    extension_models=[]

    # print("corridor_generator: got {0} extensions in this tile.".format(len(self.extensions)))

    # distribute extensions over different walls
    possible_walls=self.walls[2:]
    # check if there is a wall specified in extension list
    for (k,arg) in self.extensions:
      if not isinstance(arg, type(None)) and 'wall' in arg.keys() :
        if arg['wall'] in possible_walls: 
          possible_walls.remove(arg['wall'])
        else: #if wall is not possible remove it
          del arg['wall']
    
    # give other extensions a wall  
    for i, (k, arg) in enumerate(self.extensions):
      if k in ['panel','blocked_hole','obstacle'] and 'wall' not in arg.keys():
        try:
          arg['wall']=np.random.choice(possible_walls)
        except ValueError:
          self.extensions.remove(self.extensions[i])
        else:
          possible_walls.remove(arg['wall'])
        
    # generate element
    arg={}
    for k, arg in self.extensions:
      arg['verbose']=verbose
      element = self.generator_dic[k](**arg)
      if isinstance(element, ET.Element): extension_models.append(element)

    # rotate and translate all extension models according to position and orientation as well as width
    rotation_matrix={'+y':np.array([[1,0],[0,1]]),
          '-y':np.array([[-1,0],[0,-1]]), # turn over 180 degr
          '-x':np.array([[0,-1],[1,0]]), #turn +90degr around z (left)
          '+x':np.array([[0,1],[-1,0]])} #turn -90degr around z (right)
    thetas={'+y':0,
            '-y':3.14,
            '-x':1.57,
            '+x':-1.57}

    for m in extension_models:
      # step 1 rotate to correct orientation
      pose_element=m.find('pose')
      pose_6d=[float(v) for v in pose_element.text.split(' ')]
      pose_6d[0]*=width/2.
      pose_6d[1]*=width/2. #corridor is expected to be 2m width
      pose_6d[2]*=height/2 #corridor is expected to be 2m high, has to be rescaled to height
      position=np.array([pose_6d[0], pose_6d[1]])
      position=np.matmul(rotation_matrix[self.o], position)
      pose_6d[5]+=thetas[self.o]
      pose_6d[0]=position[0]
      pose_6d[1]=position[1]
      # step 2 translate
      pose_6d[0]+=width*self.x
      pose_6d[1]+=width*self.y
      pose_element.text = str(pose_6d[0])+' '+str(pose_6d[1])+' '+str(pose_6d[2])+' '+str(pose_6d[3])+' '+str(pose_6d[4])+' '+str(pose_6d[5])

    return extension_models


  def get_elements(self, width, height, texture, lights, visual, verbose):
    """Translate the tile to a corridor element parsed from segmented worlds
    in simulation_supervised_demo/worlds/segment.world. 
    Afterwhich heigth, width, texture is adjusted and lights is added.
    Lights are parsed from simsup_demo/lights/.
    Than position and orientation of all elements are adjusted.
    In the end the elements are given a better name.
    """
    wall_models=self.get_wall_models(width, height, texture, visual, verbose)
    light_models=self.get_lights(lights, width, height, verbose)
    extension_models = self.get_extensions(width, height, verbose)
 
    # Change name attributes so gazebo does not complain
    models=wall_models+light_models+extension_models

    names=[]
    for m in models:
      i=0
      while m.attrib['name']+"_"+str(self.x)+"_"+str(self.y)+"_"+str(i)+"_"+str(self.tile_type) in names:
        i+=1
      names.append(m.attrib['name']+"_"+str(self.x)+"_"+str(self.y)+"_"+str(i)+"_"+str(self.tile_type))
      m.attrib['name']=m.attrib['name']+"_"+str(self.x)+"_"+str(self.y)+"_"+str(i)+"_"+str(self.tile_type)

    return models

  def to_string(self):
    """display fields"""
    return "tile(x={0} ,y={1}, orientation={2}, type={3}, ext={4})".format(self.x, self.y, self.o, self.tile_type,[v[0] for v in self.extensions])  

def from_sequence_to_tiles(sequence):
  """Translate a sequence of types of tiles,
  to a sequence of tiles."""
  current_tile = Tile(0,0,'+y',0)
  visited_tiles=[current_tile]
  for step in sequence[1:]:
    current_tile = current_tile.leave(step)
    visited_tiles.append(current_tile)
  return visited_tiles

def is_feasible(sequence):
  """Compute feasibility of sequence of turns.
  Step over the sequence keepin track of current location and orientation.
  The sequence can not visit a location twice.
  """
  # 1: compute locations
  visited_tiles = from_sequence_to_tiles(sequence)
  
  # 2: compare length with length of unique list
  return len(list(set([(t.x, t.y) for t in visited_tiles]))) == len([(t.x, t.y) for t in visited_tiles])

def generate_map(length, bends):
  """ Generate a random trajectory with number of steps or tiles equal to length 
  and number of bends equal to bends.
  """
  if length < bends: bends = length

  feasible = False
  while not feasible:
    sequence = [0]
    num_bends=0
    for step in range(length):
      if num_bends < bends:
        possible_tiles = [1,2,3]
        # if all following steps has to be bends in order to get to the preset number
        if  (length-step) == (bends-num_bends):
          # print("only bends should be picked" )
          possible_tiles=[2,3]
        # Avoid three turns in the same direction in a row
        if len(sequence) > 2 and sequence[-1] == sequence[-2] and sequence[-1] in [2,3]:
          # print("2 bends of the same direction")
          if sequence[-1] == 2: 
            possible_tiles.remove(2)
          else: 
            possible_tiles.remove(3)
        tile = np.random.choice(possible_tiles)
      else: # If all bends are already used
        tile = 1
      # print("tile: ",tile)
      sequence.append(tile)
      if tile in [2,3]: num_bends +=1
    sequence.append(4)
    feasible = is_feasible(sequence)
  return sequence

def translate_map_to_element_tree(sequence, width, height, texture, lights, visual=True, extension_conf={}, verbose=False):
  """For each tile in the trajectory sequence,
  append a corresponding corridor segment on correct location.
  """
  segments=[]
  tiles=from_sequence_to_tiles(sequence)

  if extension_conf:
    # spread out a number of extensions over the different tiles
    for k in extension_conf.keys():
      if extension_conf[k]['type'] in ["passway"]: # passways are only set on straight segments
        possible_tiles=[t for t in tiles if t.tile_type == 1 and "passway" not in [e[0] for e in t.extensions]]
      elif extension_conf[k]['type'] in ["panel", "blocked_hole","obstacle"]:
        possible_tiles=[t for t in tiles if t.tile_type in [1,2,3]]
      else:
        possible_tiles=tiles
      try:
        num=extension_conf[k]['number'] # get number of occurences of this extension
      except KeyError: # in case it is not specified added to all segments
        num=len(possible_tiles)
      else:
        num=min(num,len(possible_tiles))
      print("[world_generator]: adding {0}: {2} {1}(s)".format(k, extension_conf[k]['type'], num))
      # select tiles to add extension
      indices = np.array(range(len(possible_tiles)))
      np.random.shuffle(indices) #sort the tile (except for final tile)
      for i in range(num):
        possible_tiles[indices[i]].extensions.append((extension_conf[k]['type'],extension_conf[k]['type_specific']))
          
  # Get all elements of tiles
  for tile in tiles:
    elements=tile.get_elements(width, height, texture, lights, visual, verbose)
    if verbose: print "{}".format(tile.to_string())    
    segments.extend(elements)
  return segments

def get_min_max_x_y_goal(segments):
  """ Find segment of type 4.
  Find bottom_wall of this segment.
  Return min,max of x,y of this segment.
  """
  min_x = 0
  max_x = 0
  min_y = 0
  max_y = 0
  for seg in segments:
    if seg.attrib['name'].endswith('_4') and seg.attrib['name'].startswith('wall_bottom_'):
      pose_6d=[float(v) for v in seg.find('pose').text.split(' ')]
      width = float(seg.find('link').find('collision').find('geometry').find('box').find('size').text.split(' ')[0])
      # print("Found element: {0} with pose {1},{2} and width {3}".format(seg.attrib['name'],
      #                                                                   pose_6d[0],
      #                                                                   pose_6d[1],
      #                                                                   width))
      min_x=pose_6d[0]-width/2.
      max_x=pose_6d[0]+width/2.
      min_y=pose_6d[1]-width/2.
      max_y=pose_6d[1]+width/2.
  return (min_x, max_x, min_y, max_y)

def generate_corridor(length=10,
                      bends=0,
                      width=2,
                      height=2,
                      texture='Gazebo/Grey',
                      lights='default_light',
                      visual=True,
                      save_location='',
                      extension_conf={}):
  """ The function creates a map of a possible trajectory.
  Translates the map to an xml element tree.
  Returns the tree. 
  """
  sequence=generate_map(length=length, bends=bends)
  segments=translate_map_to_element_tree(sequence,
                                        width=width,
                                        height=height,
                                        texture=texture,
                                        lights=lights,
                                        visual=visual,
                                        extension_conf=extension_conf)
  goal = get_min_max_x_y_goal(segments)
  if save_location: visualize(sequence,save_location)
  return segments, goal

if __name__ == '__main__':
  # Test the corridor generator interactively
  
  ##########
  # Test 1: Example of failure case
  # sequence = [0,1,2,2,2,4]
  # print is_feasible(sequence)
  # visualize(sequence)

  ##########
  # Test 2: Check time scale
  # durs={}
  # for l in [10,100,1000,10000]:
  #   durs[l]=[]
  #   for b in range(l):
  #     start=time.time()
  #     generate_map(l,bends=b)
  #     dur=time.time()-start
  #     print("Length {0} bends {1} duration {2}".format(l,b,dur))
  #     durs[l].append(dur)
  # # --> things get slow with length 1000 and >60 bends
  # for k in durs.keys():
  #   print k
  #   plt.plot(durs[k])
  #   plt.show()

  #########
  # Test 3: generate specific sequence and seave as new.world
  # worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
  # template_world='empty_world.world'
  # tree = ET.parse(worlds_location+template_world)
  # root = tree.getroot()
  # sequence = [0,1,2,2,1,1,3,3,1,2,4]
  # # sequence = [0]
  # segments = translate_map_to_element_tree(sequence,
  #                                       width=3,
  #                                       height=2,
  #                                       texture='Gazebo/White',
  #                                       lights='default_light')
  # goal = get_min_max_x_y_goal(segments)
  # print 'goal: ',goal
  # world=root.find('world')
  # for seg in segments:
  #   pretty_append(world, seg)
  # tree.write(os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/new.world', encoding="us-ascii", xml_declaration=True, method="xml")

  ########
  # Test 4: generate empty corridor with a panel  
  # worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
  # template_world='empty_world.world'
  # tree = ET.parse(worlds_location+template_world)
  # root = tree.getroot()
  
  # sequence = [0,1,4]

  # extension_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/extensions/'
  # extension_conf = yaml.load(open(extension_location+'config/test.yaml', 'r'))
  
  
  # segments = translate_map_to_element_tree(sequence,
  #                                       width=3,
  #                                       height=2,
  #                                       texture='Gazebo/White',
  #                                       lights='default_light',
  #                                       visual=True,
  #                                       extension_conf=extension_conf,
  #                                       verbose=True)
  # # append segments to world and write world
  # world=root.find('world')
  # for seg in segments:
  #   pretty_append(world, seg)
  # tree.write(os.environ['HOME']+'/new.world', encoding="us-ascii", xml_declaration=True, method="xml")

  # # save image
  # save_location=os.environ['HOME']+'/corridors'
  # if not os.path.isdir(save_location): os.makedirs(save_location)
  # visualize(sequence,save_location+'/'+time.strftime("%Y-%m-%d_%I-%M-%S")+'_new.jpg')

  ########
  # Test 5: generate straight corridor with passway
  worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
  template_world='empty_world.world'
  tree = ET.parse(worlds_location+template_world)
  root = tree.getroot()  
  sequence = [0,1,3,1,4]
  extension_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/extensions/'
  extension_conf = yaml.load(open(extension_location+'config/test.yaml', 'r'))  
  segments = translate_map_to_element_tree(sequence,
                                        width=3,
                                        height=3,
                                        texture='Gazebo/White',
                                        lights='default_light',
                                        visual=True,
                                        extension_conf=extension_conf,
                                        verbose=True)
  # append segments to world and write world
  world=root.find('world')
  for seg in segments:
    pretty_append(world, seg)
  tree.write(os.environ['HOME']+'/new.world', encoding="us-ascii", xml_declaration=True, method="xml")

  # save image
  # save_location=os.environ['HOME']+'/corridors'
  # if not os.path.isdir(save_location): os.makedirs(save_location)
  # visualize(sequence,save_location+'/'+time.strftime("%Y-%m-%d_%I-%M-%S")+'_new.jpg')

  print 'done'
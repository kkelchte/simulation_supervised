#!/usr/bin/python
import xml.etree.ElementTree as ET
import numpy as np
import os,sys, time
import argparse

import yaml

import matplotlib.pyplot as plt

from corridor_generator import generate_corridor

"""
Author: Klaas Kelchtermans

world_generator.py: generates a random corridor as a world file (later possibly also a top down view) based on the extensions.
Steps:
1. parse recipe from arguments and yaml configuration file
2. pull in template world file
3. In case of corridor: add a corridor coming from corridor_generator
4. Add extensions according to recipe
5. write world and config file to destination folder

Dependencies are:

"""
#############
# Initialize variables
np.random.seed(int(time.time()))

#############
# Define utility functions
def pretty_append(world, element):
  """
  Append a model to the world but ensure the indentation is done correctly.
  """
  normal_tail=world[-2].tail # get normal indentation-tail ex '\n    '
  last_tail=world[-1].tail # get indentation-tail for last element ex '\n  '
  world[-1].tail = normal_tail # give current last element the correct normal indentation-tail
  world.append(element) # add new last element
  world[-1].tail = last_tail # give new last element correct last indentation-tail

#############
# 1. parse recipe from arguments and yaml configuration file
parser = argparse.ArgumentParser(description="""world_generator.py: generates a random corridor as a world file (later possibly also a top down view) based on the extensions.""")
#   ------  Corridor arguments
parser.add_argument("--corridor_type", default='normal', type=str, help="default is normal which is a bended corridor. Empty means wall parts of segments will not have a visible part.")
parser.add_argument("--corridor_length", default=10, type=int, help="Defines number of tiles the corridor long (not used in case of empty type of corridor).")
parser.add_argument("--corridor_width", default=3, type=float, help="Defines width of the corridor (not used in case of empty type of corridor).")
parser.add_argument("--corridor_height", default=3, type=float, help="Defines height of the corridor (not used in case of empty type of corridor).")
parser.add_argument("--corridor_bends", default=0, type=int, help="Defines the number of turns the corridor should make over its length. Combined with the length it specifies the density of bends.")
parser.add_argument("--lights", default='default_light', type=str, help="Defines the type of light added in each segment of the corridor according to simsup_demo/lights/.")
parser.add_argument("--texture", default='Gazebo/White', type=str, help="Defines the texture of the walls of the corridor according to gazebo.material.")
#   ------  Extension arguments --> specified with nested yaml file
parser.add_argument("--extension_config", default='empty', type=str, help="The extensions are specified in a nested yaml file within dir simsup_demo/extensions/config/{name}.yaml. For each found extension the extension is added at random locations in the corridor.")
#   ------  Settings
parser.add_argument("--output_dir", default='', type=str, help="The directory where the world and background file is saved.")
parser.add_argument("--output_file", default='new', type=str, help="The directory where the world and background file is saved.")

FLAGS, others = parser.parse_known_args()

# make output directory an absolute path or default path
if len(FLAGS.output_dir) == 0:
  FLAGS.output_dir = os.environ['HOME']
  # FLAGS.output_dir = os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds'
elif not FLAGS.output_dir.startswith('/'):
  FLAGS.output_dir = os.environ['HOME']+'/'+FLAGS.output_dir


# parse extension-configuration file
extension_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/extensions/'

conf = yaml.load(open(extension_location+'config/'+FLAGS.extension_config+'.yaml', 'r'))
if not conf: conf={}
print("\n[world_generator.py] Settings:")
for f in FLAGS.__dict__: print("{0}: {1}".format( f, FLAGS.__dict__[f]))

#############
# 2. pull in template world file
worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
template_world='empty_world.world'
tree = ET.parse(worlds_location+template_world)
root = tree.getroot()
world = root.find('world')

#############
# 3. For each extension in conf load the elements in list
# extensions=[]

#############
# 4. Build a corridor and add extensions:
# returns all segments (models) to be added to the tree
segments=[]

# save output image in 'corridors'
if not os.path.isdir(FLAGS.output_dir+'/corridors'): os.makedirs(FLAGS.output_dir+'/corridors')
segments, goal = generate_corridor(length=FLAGS.corridor_length,
                            bends=FLAGS.corridor_bends,
                            width=FLAGS.corridor_width,
                            height=FLAGS.corridor_height,
                            texture=FLAGS.texture,
                            lights=FLAGS.lights,
                            visual=FLAGS.corridor_type=='normal',
                            extension_conf=conf,
                            save_location=FLAGS.output_dir+'/corridors/'+time.strftime("%Y-%m-%d_%I-%M-%S")+'_'+FLAGS.output_file+'.png')

# add them to the world file
for seg in segments:
    pretty_append(world, seg)

#############
# 5. write world and config file
# write config file
# adjust delay_evaluation according to
env_conf= {'delay_evaluation': 5, #might have to be updated if many extensions introduce delays
        'min_depth': 0.2,
        'goal_min_x': goal[0],
        'goal_max_x': goal[1],
        'goal_min_y': goal[2],
        'goal_max_y': goal[3]}

yaml.dump(env_conf, open(FLAGS.output_dir+'/'+FLAGS.output_file+'.yaml', 'w'), default_flow_style=False)
tree.write(FLAGS.output_dir+'/'+FLAGS.output_file+'.world', encoding="us-ascii", xml_declaration=True, method="xml")
print("[world_generator.py]: created {0}/{1}".format(FLAGS.output_dir, FLAGS.output_file))

# save world file with time_tag in case we need it
if not os.path.isdir(FLAGS.output_dir+'/worlds'): os.makedirs(FLAGS.output_dir+'/worlds')
tree.write(FLAGS.output_dir+'/worlds/'+time.strftime("%Y-%m-%d_%I-%M-%S")+'_'+FLAGS.output_file+'.world', encoding="us-ascii", xml_declaration=True, method="xml")

#!/usr/bin/python
import xml.etree.ElementTree as ET
#from lxml import etree as ET
import os,  re, sys, shutil, time
from copy import deepcopy
import random
# from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np
# if len(sys.argv) > 1:
#   result_world=sys.argv[1]
# else:
result_world='sandbox.world'

np.random.seed(int(time.time()))
random.seed(int(time.time()))

# example file that is adapted:
template_world='template.world'
worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
if len(sys.argv) >= 2:
  location=sys.argv[1]+'/'
else:
  location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
# print('[generator] World and image is saved in: {}'.format(location))

tree = ET.parse(worlds_location+template_world)
root = tree.getroot()
world = root.find('world')

# place ligths: between 1:20 lights on random locations [-10:10, -10:10, 0.5:3.9]
number_of_lights=random.choice(range(1,15))
for l in range(number_of_lights):
  light=ET.SubElement(world,'light')
  light.set('type','diffuse')
  light.set('name','lightball_'+str(l))
  cast_shadows=ET.SubElement(light,'cast_shadows')
  cast_shadows.text='true'
  diffuse=ET.SubElement(light,'diffuse')
  diffuse.text='0.8 0.8 0.8 1'
  specular=ET.SubElement(light,'specular')
  specular.text='0.2 0.2 0.2 1'
  attenuation=ET.SubElement(light,'attenuation')
  rangel=ET.SubElement(attenuation,'range')
  rangel.text='100'
  constant=ET.SubElement(attenuation,'constant')
  constant.text='0.7'
  linear=ET.SubElement(attenuation,'linear')
  linear.text='0.01'
  quadratic=ET.SubElement(attenuation,'quadratic')
  quadratic.text='0.001'
  direction=ET.SubElement(light,'direction')
  direction.text='-0.5 0.1 -0.9'
  pos=ET.SubElement(light,'pose')
  x=random.uniform(-10,10)
  y=random.uniform(-10,10)
  z=random.uniform(0.5,3.9)
  pos.text=str(x)+' '+str(y)+' '+str(z)+' 0 0 0'


# put texture on the walls
texture_file=worlds_location+'../../simulation_supervised_tools/etc/textures.txt'
texture_list=[ t[:-1] for t in open(texture_file).readlines()]
# print "texture_list "+str(texture_list)

surrounding=[m for m in world.findall('model') if m.get('name')=='four_walls'][0]
# print "surrounding "+str(surrounding)

for link in surrounding.findall('link'):
  material=link.find('visual').find('material').find('script').find('name')
  material.text="Gazebo/"+random.choice(texture_list)

# place objects from list
# add models from file to list if the model exists in .gazebo/models
model_file=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_tools/etc/models.txt'
model_list=[ m[:-1] for m in open(model_file).readlines() if os.path.exists(worlds_location+'../models/'+m[:-1])]
# print "model_list "+str(model_list)

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
# ax.set_title('Trajectory in sandbox')

density=4
for x in range(-7,10,density):
  #for x in range(-3,4,3):
  for y in range(-7,10,density):
    #for y in range(-3,4,3):
    if abs(x)<2 and abs(y)<2:
      continue
    model_name = random.choice(model_list)
    #print(model_name, ' on ',x,', ',y)
    incl=ET.SubElement(world, 'include')
    ure=ET.SubElement(incl,'uri')
    ure.text='model://'+model_name
    pos=ET.SubElement(incl,'pose')
    yaw=random.uniform(0,3.14)
    d=0.8
    xd=random.uniform(-d,d)
    yd=random.uniform(-d,d)
    pos.text=str(x-0.5+xd)+' '+str(y-0.5+yd)+' 0 0 0 '+str(yaw)
    plt.scatter(x-0.5+xd, y-0.5+yd, s=1000, marker=np.random.choice(['o','v', '^', '<', '>', '8', 's', 'p', '*', 'h']),alpha=0.5 )
plt.plot(0,0, marker='o', color='black', ls='')
# plt.plot(10,10, marker='o', color='black', ls='')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.axis('off')

# plt.show()
# name sandbox.png is used by ground_truth_listener to visualize trajectory (!)
plt.savefig(location+'sandbox.png', bbox_inches='tight')

print('make: {}'.format(result_world))
tree.write(os.path.join(location,result_world), encoding="us-ascii", xml_declaration=True, method="xml")

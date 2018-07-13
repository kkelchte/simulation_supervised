#!/usr/bin/python
from ou_noise import OUNoise 
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import os,sys, time

np.random.seed(int(time.time()))


number_of_walls=150
width = 3.0 #1
height = 4 #2
result_world='canyon.world'
show_plan=False
if len(sys.argv) >= 2:
  location=sys.argv[1]+'/'
else:
  location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'

# print('[generator] World and image is saved in: {}'.format(location))
worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'

walls=[]
fig = plt.figure(figsize=(10, 10))
axes = plt.gca()
axes.set_xlim([-25,25])
axes.set_ylim([0,75])
# axes.set_title('Trajectory in canyon')

noise = OUNoise(1,theta=0.3,sigma=0.5)
alpha=0    
for w in range(number_of_walls):
    l=np.random.uniform(0.3,0.7)
    alpha=np.pi/6*noise.noise()[0] #np.random.uniform(-np.pi/3,np.pi/3)
    # l =-(np.pi/6)/(0.2-0.01)*np.abs(alpha)
    # print(l)
    # print(type(l))
    # import pdb; pdb.set_trace()
    # l=1
    if w == 0:
        xo=-width/2.
        yo=0
    else:
        xo=xe
        yo=ye
    
    xe=xo+l*np.sin(alpha)
    ye=yo+l*np.cos(alpha)
    xc=xo+l*np.sin(alpha)/2.
    yc=yo+l*np.cos(alpha)/2.
    
    walls.extend([([xc,yc],l,alpha), ([xc+width,yc],l,alpha)])

    plt.plot([xo,xe],[yo,ye],color='grey')
    plt.plot([xo+width,xe+width],[yo,ye], color='grey')

plt.plot(0,0, marker='o', color='black', ls='')
# plt.plot(10,10, marker='o', color='black', ls='')
plt.axis('off')

# name canyon.png is used by ground_truth_listener to visualize trajectory (!)
plt.savefig(location+'canyon.png', bbox_inches='tight')

if show_plan:
    plt.plot([0,0],[0,5],color='k')
    plt.show()

template_world='empty_world_with_wall.world'
tree = ET.parse(worlds_location+template_world)
root = tree.getroot()
world = root.find('world')

for i,w in enumerate(walls):
    model=ET.SubElement(world, 'model', attrib={'name': 'wall_'+str(i)})
    static=ET.SubElement(model,'static')
    static.text='1'
    pos=ET.SubElement(model,'pose')
    yaw=-w[2]
    pos.text= str(w[0][0])+' '+str(w[0][1])+' '+str(height/2)+' 0 0 '+str(yaw)    
    link=ET.SubElement(model, 'link', attrib={'name':'link'})
    collision = ET.SubElement(link, 'collision', attrib={'name':'collision'})
    visual = ET.SubElement(link, 'visual', attrib={'name':'visual'})
    material = ET.SubElement(visual,'material')
    script = ET.SubElement(material,'script')
    name = ET.SubElement(script,'name')
    name.text='Gazebo/Grey'
    uri = ET.SubElement(script,'uri')
    uri.text='file://media/materials/scripts/gazebo.material'
    for element in [collision,visual]: 
        geo = ET.SubElement(element, 'geometry')
        box = ET.SubElement(geo, 'box')
        size = ET.SubElement(box, 'size')
        size.text='0.02 '+str(w[1])+' '+str(height)

print('saved {1}{0} '.format(result_world, location))
tree.write(os.path.join(location,result_world), encoding="us-ascii", xml_declaration=True, method="xml")

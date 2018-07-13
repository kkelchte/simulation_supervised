#!/usr/bin/python
import xml.etree.ElementTree as ET
# import xml.dom.minidom
import numpy as np
import matplotlib.pyplot as plt
import os,sys, time

np.random.seed(int(time.time()))


result_world='forest.world'
worlds_location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'

if len(sys.argv) >= 2:
  location=sys.argv[1]+'/'
else:
  location=os.environ['HOME']+'/simsup_ws/src/simulation_supervised/simulation_supervised_demo/worlds/'
# print('[generator] World and image is saved in: {}'.format(location))

# Size of total field
W = 120 # depth
D = 120 #200 # width
density = 0.20

NW = int(W*density) # number of trees over width
ND = int(D*density) # number of trees over depth
# ND = 30 # number of trees over depth

# get average locations for trees
trees = [(int(W*i/NW)-W/2+np.random.uniform()*W/NW,int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]
# trees = [(int(W*i/NW)-W/2+np.random.uniform()*W/NW,int(D*j/ND)+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]

# show trees
x = [t[0] for t in trees if abs(t[0])>= 4 or abs(t[1]) >= 4]
y = [t[1] for t in trees if abs(t[0])>= 4 or abs(t[1]) >= 4]
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
# ax.set_title('Trajectory in forest')
plt.scatter(x, y, alpha=0.5)
plt.plot(0,0, marker='o', color='black', ls='')
# plt.plot(10,10, marker='o', color='black', ls='')

plt.xlim(-60, 60)
plt.ylim(-60, 60)
plt.axis('off')

# name forest.png is used by ground_truth_listener to visualize trajectory (!)
plt.savefig(location+'forest.png', bbox_inches='tight')

# location='../../simulation_supervised_demo/worlds/'
template_world='empty_world.world'
tree = ET.parse(worlds_location+template_world)
root = tree.getroot()
world = root.find('world')

for t in trees:
	if abs(t[0])< 4 and abs(t[1]) < 4:
		continue
	incl=ET.SubElement(world, 'include')
	name=ET.SubElement(incl, 'name')
	name.text='tree_'+str(t[0])+'_'+str(t[1])
	ure=ET.SubElement(incl,'uri')
	ure.text='model://tree'
	pos=ET.SubElement(incl,'pose')
	pos.text= str(t[0])+' '+str(t[1])+' '+str(2)+' 0 0 0'    

# xml = xml.dom.minidom.parse(tree) # or xml.dom.minidom.parseString(xml_string)
# pretty_xml_as_string = xml.toprettyxml()

print('make: {}'.format(result_world))

tree.write(os.path.join(location,result_world), encoding="us-ascii", xml_declaration=True, method="xml")
# f=open(os.path.join(location,result_world),'w')
# f.write(pretty_xml_as_string)
# f.close()
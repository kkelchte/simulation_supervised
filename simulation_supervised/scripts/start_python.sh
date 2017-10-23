#!/bin/bash
source ~/tensorflow/bin/activate
export PYTHONPATH=$HOME/tensorflow/lib/python2.7:$HOME/tensorflow/lib/python2.7/site-packages:$HOME/tensorflow/examples:\
$HOME/driver_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:$HOME/catkin_ws/devel/lib/python2.7/dist-packages\
:/usr/lib/python2.7/dist-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-8.0/lib64:/usr/local/cudnn/lib64:
cd $HOME/tensorflow/pilot/pilot
echo "python main.py $@"
python main.py $@  


#!/bin/bash
source ~/tensorflow/bin/activate
export PYTHONPATH=$HOME/tensorflow/lib/python2.7:$HOME/tensorflow/lib/python2.7/site-packages:$HOME/tensorflow/examples:\
$HOME/drone_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:$HOME/simsup_ws/devel/lib/python2.7/dist-packages\
:/usr/lib/python2.7/dist-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/usr/local/cudnn/lib64:/usr/local/cuda/lib64:/opt/ros/kinetic/lib
export PATH=$PATH:/usr/local/cuda/bin

cd $HOME/tensorflow/pilot/pilot
python_command="python main.py --online  $@"
echo "start_python.sh:"
echo $python_command
$python_command

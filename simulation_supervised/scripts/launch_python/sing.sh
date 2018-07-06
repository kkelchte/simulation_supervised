#!/bin/bash
export PYTHONPATH=$HOME/tensorflow/pilot:\
$HOME/drone_ws/devel/lib/python2.7/dist-packages:\
/opt/ros/kinetic/lib/python2.7/dist-packages:\
$HOME/simsup_ws/devel/lib/python2.7/dist-packages:\
/usr/lib/python2.7/dist-packages:\
/usr/local/lib/python2.7/dist-packages

export LD_LIBRARY_PATH=$HOME/simsup_ws/devel/lib:\
$HOME/drone_ws/devel/lib:/opt/ros/kinetic/lib:\
/usr/local/cuda/lib64:/usr/local/cudnn/lib64:\
/.singularity.d/libs

python $@

echo 'singularity: done'
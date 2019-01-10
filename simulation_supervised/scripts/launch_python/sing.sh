#!/bin/bash
# export PYTHONPATH=$HOME/tensorflow/pilot:\
# $HOME/drone_ws/devel/lib/python2.7/dist-packages:\
# /opt/ros/kinetic/lib/python2.7/dist-packages:\
# $HOME/simsup_ws/devel/lib/python2.7/dist-packages:\
# /usr/lib/python2.7/dist-packages:\
# /usr/local/lib/python2.7/dist-packages

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/.singularity.d/libs/:/usr/local/cuda/lib64:/usr/local/cudnn/lib64
export TF_CPP_MIN_LOG_LEVEL=3

python $@

echo 'singularity: done'
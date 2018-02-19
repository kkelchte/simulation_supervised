#!/bin/bash
export PYTHONPATH=$HOME/tensorflow/q-learning:\
$HOME/drone_ws/devel/lib/python2.7/dist-packages:\
/opt/ros/kinetic/lib/python2.7/dist-packages:\
$HOME/simsup_ws/devel/lib/python2.7/dist-packages:\
/usr/lib/python2.7/dist-packages:\
/usr/local/lib/python2.7/dist-packages

export LD_LIBRARY_PATH=$HOME/simsup_ws/devel/lib:\
$HOME/drone_ws/devel/lib:/opt/ros/kinetic/lib:\
/usr/local/cuda-8.0/lib64:/usr/local/cudnn/lib64:\
/.singularity.d/libs

cd $HOME/tensorflow/q-learning/pilot
python_command="python main.py --offline False  $@"
echo "start_python_sing_ql.sh:"
echo $python_command
$python_command


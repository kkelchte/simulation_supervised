#!/bin/bash

echo "Args: $@"
export VGL_VERBOSE=1
export LIBGL_DEBUG=verbose

echo ----------- Check environment
printenv
echo ----------- Check xpra
ps -ef | grep xpra
echo ----------- Check GLX 
xdpyinfo | grep GL
echo ----------- Check nvidia-lib 
ls /usr/local/nvidia
ls /usr/local/nvidia/lib64
echo ----------- Check ROSCORE
which roscore
roscore &
sleep 20
ps -ef | grep ROS
echo ----------- Check Gazebo
which gzserver
gzserver --verbose &
sleep 10
ps -ef | grep gz
echo ------------ Done

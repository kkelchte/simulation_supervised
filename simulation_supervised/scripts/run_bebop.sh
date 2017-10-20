#!/bin/bash
######################################################
# Settings:
# $1=TAG
# $2=MODELDIR
######################################################
if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
if [ -z $2 ] ; then
  # MODELDIR="eva_onl_mobsm_lr05_hub_6"
  # MODELDIR="onl_pre_mob_auxod_1s"
	MODELDIR="offl_mobsm_sameseed_auxd_2"
else
	MODELDIR="$2"
fi
######################################################
# Launch ROS if it hasn't started yet:
mkdir -p $HOME/tensorflow/log/$TAG
LLOC=$HOME/tensorflow/log/$TAG
if [ $(ps -ef | grep rosmaster | wc -l) -lt 2 ] ; then 
	echo 'launching ROS...';
	COMMANDR="roslaunch simulation_supervised_demo bebop_real.launch saving_location:=$LLOC "
	xterm -hold -e $COMMANDR &
	pidros=$!
  # roslaunch simulation_supervised_demo bebop_real.launch & 
fi
kill_combo(){
  echo "kill ros:"
  killall -9 roscore >/dev/null 2>&1 
  killall -9 rosmaster >/dev/null 2>&1
  killall -9 /*rosout* >/dev/null 2>&1 
  kill -9 $pidros >/dev/null 2>&1
  while kill -0 $pidpython;
  do      
    kill $pidpython >/dev/null 2>&1
    sleep 0.05
  done
  sleep 10
}
######################################################
finished=false
while [ $finished != true ] ; do
  mkdir -p $HOME/tensorflow/log/$TAG
  cd $HOME/tensorflow/log/$TAG
  python_script="start_python.sh"
  LOGDIR="$TAG/$(date +%F_%H%M)_real"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  start_python(){
    ARGUMENTS="--off_policy True --show_depth True --show_odom False --continue_training True --extra_control_layer True --evaluate True --log_tag $LOGDIR --checkpoint_path $MODELDIR --load_config True"
    COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
    echo $COMMANDP
    xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H%M%S) -hold -e $COMMANDP &
    pidpython=$!
    echo "PID Python tensorflow: $pidpython"
    for i in $(seq 10) ; do 
      echo "wait: $i / 10"
      sleep 1 #wait some seconds for model to load otherwise you miss the start message
    done  
  }
  start_python
  read -p 'you want more? [y]/n: ' answer
  if [ -z $answer ] ; then
    read -p 'TAG? ' TAG
    ls -t $HOME/tensorflow/log
    read -p 'MODELDIR? ' MODELDIR
  else
    if [ $answer == 'n' ] ; then
      echo 'finished then.'
      finished=true
    else
      read -p 'TAG? ' TAG
      ls -t $HOME/tensorflow/log
      read -p 'MODELDIR? ' MODELDIR
    fi
  fi
done
kill_combo
echo 'done'
date +%F_%H%M%S



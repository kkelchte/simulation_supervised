#!/bin/bash
if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
if [ -z $2 ] ; then
  # MODELDIR="offline_esat_rand_recovery"
  MODELDIR="offline_esat_ou_aux"
else
	MODELDIR="$2"
fi
if [ ! -e /home/klaas/tensorflow/log/$MODELDIR/checkpoint ] ; then
	for d in $(ls /home/klaas/tensorflow/log/$MODELDIR) ; do
		# echo $d
		if [[ -d /home/klaas/tensorflow/log/$MODELDIR/$d  && -e /home/klaas/tensorflow/log/$MODELDIR/$d/checkpoint ]] ; then 
			LAST=$d;
		fi; 
	done
	MODELDIR=$MODELDIR/$LAST
	echo "Changed modeldir to: $MODELDIR"
fi
if [ -z $3 ] ; then
	BAG="2017-07-04-17-25-09.bag"
else
	BAG="$3"
fi

if [ -z $4 ] ; then
	PARAMS="--auxiliary_depth True"
else
	PARAMS="${@:4}"
fi
######################################################
echo 'launching ROS...';
COMMANDR="roslaunch simulation_supervised load_params.launch drone_config:=bebop_real.yaml"
xterm -hold -e $COMMANDR& 
pidros=$!

sleep 5
######################################################
echo 'launching show_control...';
COMMANDS="roslaunch simulation_supervised_tools show_control.launch"
xterm -hold -e $COMMANDS& 
pidshow=$!

######################################################
# Start tensorflow with command defined above
# mkdir -p /esat/qayd/kkelchte/tensorflow/online_log/$TAG/log
mkdir -p $HOME/tensorflow/log/$TAG
# mkdir -p $HOME/tensorflow/log/$TAG
# cd /esat/qayd/kkelchte/tensorflow/online_log/$TAG/log
cd $HOME/tensorflow/log/$TAG
echo "RUNNING LOCALLY"
python_script="start_python.sh"
start_python(){
  LOGDIR="$TAG/$(date +%F_%H%M)"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  ARGUMENTS="--off_policy True --show_depth True --real True --evaluate True --continue_training True --log_tag $LOGDIR --checkpoint_path $MODELDIR $PARAMS"
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H%M%S) -hold -e $COMMANDP &
  pidpython=$!

  sleep 10 #wait some seconds for model to load otherwise you miss the start message  
}
start_python

#####################################################
echo 'publish ready'
xterm -hold -e "$(rostopic pub $(rosparam get ready) std_msgs/Empty)" &
pidpub=$!
#####################################################
echo "Start rosbag"
rosbag play ~/rosbags/$BAG

kill $pidpython $pidshow $pidros $pidpub
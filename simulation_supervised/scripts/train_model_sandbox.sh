#!/bin/bash

######################################################
# Settings:
# $1=TAG
# $2=MODELDIR
# $3=NUMBER_OF_FLIGTHS
# $4=START_INDEX
# $5=PARAMS
######################################################
# Define training settings
# TAG="testing" #"pretrained_esat_30fps" # #"wd_$WEIGHT_DECAY-bs_$BUFFER_SIZE"
if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
if [ -z $2 ] ; then
  MODELDIR="offline_esat_ou_aux_joint"
else
  MODELDIR="$2"
fi
if [ -z $3 ] ; then
  NUMBER_OF_FLIGHTS=3
else
  NUMBER_OF_FLIGHTS="$3"
fi
if [ -z $4 ] ; then
  START_INDEX=0
else
  START_INDEX="$4"
fi

PARAMS="${@:5}"
WORLDS=(sandbox)
COUNTSUC=(0) 
COUNTTOT=(0)

# PARAMS="--depth_input True --continue_training False --network fc_control --auxiliary_depth False"
echo "+++++++++++++++++++++++TRAIN+++++++++++++++++++++"
echo "TAG $TAG"
echo "MODELDIR $MODELDIR"
echo "NUMBER OF FLIGHTS ${NUMBER_OF_FLIGHTS}"
echo "PARAMS ${PARAMS[@]}"

echo "WORLDS ${WORLDS[@]}"

RANDOM=125 #seed the random sequence

#check if you re running condor
docker=true
echo "condor slot: $_CONDOR_SLOT"
if [ -z $_CONDOR_SLOT ] ; then
  docker=false
fi
echo "Docker: $docker"

RECOVERY=false

######################################################
# Start roscore and load general parameters
start_ros(){
  echo "start ROS"
  roslaunch simulation_supervised load_params.launch global_param:=online_param.yaml&
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 10  
}
start_ros

######################################################
# Start tensorflow with command defined above
# mkdir -p /esat/qayd/kkelchte/tensorflow/online_log/$TAG/log
mkdir -p $HOME/tensorflow/log/$TAG
# mkdir -p $HOME/tensorflow/log/$TAG
# cd /esat/qayd/kkelchte/tensorflow/online_log/$TAG/log
cd $HOME/tensorflow/log/$TAG
# cd $HOME/tensorflow/log/$TAG
if [ $docker = true ] ; then
  echo "RUNNING IN DOCKER"
  python_script="start_python_docker.sh"
else 
  echo "RUNNING LOCALLY"
  python_script="start_python.sh"
fi
start_python(){
  LOGDIR="$TAG/$(date +%F_%H%M)_train"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  ARGUMENTS="--log_tag $LOGDIR --checkpoint_path $MODELDIR $PARAMS"
  if [ $RECOVERY = true ] ; then
    ARGUMENTS="$ARGUMENTS --recovery_cameras True"
  fi
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H%M%S) -hold -e $COMMANDP &
  pidpython=$!
  echo "PID Python tensorflow: $pidpython"
  sleep 20 #wait some seconds for model to load otherwise you miss the start message  
}
start_python
# Start ros with launch file
kill_combo(){
  echo "kill ros:"
  kill -9 $pidlaunch >/dev/null 2>&1 
  killall -9 roscore >/dev/null 2>&1 
  killall -9 rosmaster >/dev/null 2>&1
  killall -9 /*rosout* >/dev/null 2>&1 
  killall -9 gzclient >/dev/null 2>&1
  kill -9 $pidros >/dev/null 2>&1
  while kill -0 $pidpython;
  do      
    kill $pidpython >/dev/null 2>&1
    sleep 0.05
  done
  sleep 10
}

crash_number=0
#location for logging
mkdir $LLOC/xterm_log

# for i in $(seq 0 $((NUMBER_OF_FLIGHTS-1))) ;
i=$START_INDEX
while [[ $i -lt $((START_INDEX+NUMBER_OF_FLIGHTS)) ]] ;
do
  NUM=$((i%${#WORLDS[@]}))
  # If it is not esat simulated, you can create a new world
  EXTRA_ARGUMENTS=""
  if [[ ${WORLDS[NUM]} != esat* ]] ; then
    python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $LLOC
    EXTRA_ARGUMENTS=" background:=$LLOC/${WORLDS[NUM]}.png world_name:=$LLOC/${WORLDS[NUM]}.world"
  fi
  if [ ${WORLDS[NUM]} == sandbox ] ; then
    SANDLOC=$(rospack find simulation_supervised_demo)/worlds/sandboxes_train
    EXTRA_ARGUMENTS=" background:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).png world_name:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).world"
  fi
  crashed=false
  # Clear gazebo log folder to overcome the impressive amount of log data
  if [ $((i%50)) = 0 ] ; then rm -r $HOME/.gazebo/log/* ; fi
  if [ ! -d $LLOC ] ; then echo 'esat is unmounted so stop.' ; exit ; fi
  echo "$(date +%H:%M) -----------------------> Started with run: $i crash_number: $crash_number"
  
  # x=$(awk "BEGIN {print -1+2*$((RANDOM%=100))/100}")
  # y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
  # z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
  # Y=$(awk "BEGIN {print 1.57-0.25+0.5*$((RANDOM%=100))/100}")
  speed=1.3
  x=0
  y=0
  z=0.5
  Y=1.57  
  LAUNCHFILE="${WORLDS[NUM]}.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y starting_height:=$z speed:=$speed log_folder:=$LLOC\
   recovery:=$RECOVERY $EXTRA_ARGUMENTS"
  echo $COMMANDR
  START=$(date +%s)     
  xterm -l -lf $LLOC/xterm_log/run_${i}_$(date +%F_%H%M%S) -hold -e $COMMANDR &
  pidlaunch=$!
  echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
  echo "Run started in xterm: $pidlaunch"
  while kill -0 $pidlaunch; 
  do 
    if [ docker = true ] ; then 
      # if [[ $NEW_TOT_SUS != $TOTAL_SUS || $(( $(date +%s) - START - DIFF)) -gt 30 ]] ; then
      # if the last check was too long ago you probably got suspended
      if [[ $(( $(date +%s) - START - DIFF)) -gt 30 ]] ; then
        sleep 30 #wait for big tick to update
        # NEW_TOT_SUS="$(condor_q -glob -l $cluser_id | grep TotalSuspensions | tail -1 | cut -d ' ' -f 3)"
        TOTAL_SUS="$(condor_q -glob -l $cluser_id | grep TotalSuspensions | tail -1 | cut -d ' ' -f 3)"
        echo "I was suspended for the $TOTAL_SUS 'th time."
        START=$(( $(date +%s) - $DIFF ))
      fi
    fi
    END=$(date +%s)
    DIFF=$(( $END - $START ))
    if [ $DIFF -gt 300 ] ; 
    then
      if [ $crash_number -ge 3 ] ; then 
        echo "$(date +%H:%M) ########################### KILLED ROSCORE" >> $LLOC/crash      
        echo "$(date +%H:%M) ########################### KILLED ROSCORE"     
        sleep 0.5
        kill_combo
        crash_number=0
        echo "restart python:"
        if [ "$(ls $LLOC | wc -l)" -ge 6 ] ; then
          MODELDIR="$LLOC"
        else 
          rm -r $LLOC
        fi
        start_ros
        PARAMS="$(echo $PARAMS | sed 's/--continue_training False/--continue_training True/') --continue_training True"
        start_python
      else
        echo "$(date +%H:%M) #### KILLED ROSLAUNCH: $crash_number" >> $LLOC/crash
        echo "$(date +%H:%M) #### KILLED ROSLAUNCH: $crash_number"
        sleep 0.5
        kill -9 $pidlaunch >/dev/null 2>&1
        sleep 2
        crash_number=$((crash_number+1))
        crashed=true
      fi
    fi
    sleep 0.1
  done
  if [[ crashed != true ]] ; then
    i=$((i+1))
    if [ $(tail -1 $LLOC/log) == 'success' ] ; then
      COUNTSUC[NUM]="$((COUNTSUC[NUM]+1))"
    fi
    COUNTTOT[NUM]="$((COUNTTOT[NUM]+1))"
    echo "$(date +%F_%H-%M) finished run $i in world ${WORLDS[NUM]} with $(tail -1 ${LLOC}/log) resulting in ${COUNTSUC[NUM]} / ${COUNTTOT[NUM]}"
    # if [[ $((i%5)) = 0 && $crashed = false ]] ; then
    #   python $HOME/catkin_ws/src/simulation_supervised/simulation_supervised_tools/python/viz_trajectories.py $LOGDIR True &
    # fi  
    sleep 5
  fi
  
done
kill_combo
echo 'done'
date +%F_%H%M%S





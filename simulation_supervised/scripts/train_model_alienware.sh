#!/bin/bash

######################################################
# Define training settings
# TAG="testing" #"pretrained_esat_30fps" # #"wd_$WEIGHT_DECAY-bs_$BUFFER_SIZE"
if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
# MODELDIR="2017-04-23_1016_esat_cont_depth0420_1514"
# MODELDIR="poc_alienware/2017-06-03_0338"
# MODELDIR="train_esat_pretrained_esat_noisy_grad_desc_lre3_epe1_alpe2_veryfast/2017-05-03_0305"
if [ -z $2 ] ; then
  MODELDIR="2017-04-23_1016_esat_cont_depth0420_1514"
else
  MODELDIR="$2"
fi
WORLDS=(esat_v1) 
if [[ "$3" = "2" ]] ; then
  WORLDS=(esat_v1 esat_v2)
fi
# PARAMS="${@:4}"
PARAMS="--recovery_cameras True --owr True" #--auxiliary_depth True 
echo "TAG $TAG"
echo "MODELDIR $MODELDIR"
echo "WORLDS ${WORLDS[@]}"
echo "PARAMS ${PARAMS[@]}"

NUMBER_OF_FLIGHTS=1000
RANDOM=125 #seed the random sequence

#Define whether model is trained in docker image
DOCKER=false #impact on start_python script and logging xterms

######################################################
# Start roscore and load general parameters
start_ros(){
  echo "start ROS"
  roslaunch simulation_supervised load_params.launch global_param:=online_param.yaml&
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 5  
}
start_ros

######################################################
# Start tensorflow with command defined above
# mkdir -p /esat/qayd/kkelchte/tensorflow/online_log/$TAG
mkdir -p /home/klaas/tensorflow/log/$TAG
if [ $DOCKER = true ] ; then
  echo "RUNNING IN DOCKER"
  python_script="start_python_docker.sh"
else 
  echo "RUNNING LOCALLY"
  python_script="start_python.sh"
fi
start_python(){
  LOGDIR="$TAG/$(date +%F_%H%M)"
  ARGUMENTS="--continue_training True --log_tag $LOGDIR --checkpoint_path $MODELDIR $PARAMS"
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  if [ $DOCKER = true ] ; then 
    xterm -l -hold -e $COMMANDP &
  else
    xterm -hold -e $COMMANDP &
  fi
  pidpython=$!
  echo "PID Python tensorflow: $pidpython"
  sleep 10 #wait some seconds for model to load otherwise you miss the start message  
}
start_python
# Start ros with launch file
kill_combo(){
  echo "kill ros:"
  kill -9 $pidlaunch >/dev/null 2>&1 
  killall -9 roscore >/dev/null 2>&1 
  killall -9 rosmaster >/dev/null 2>&1
  killall -r /*rosout* >/dev/null 2>&1 
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
LLOC="/home/klaas/tensorflow/log/$LOGDIR"
# LLOC="/esat/qayd/kkelchte/tensorflow/online_log/$LOGDIR"
for i in $(seq 0 $((NUMBER_OF_FLIGHTS-1))) ;
do
  crashed=false
  # Clear gazebo log folder to overcome the impressive amount of log data
  if [ $((i%50)) = 0 ] ; then rm -r $HOME/.gazebo/log/* ; fi
  NUM=$((i%${#WORLDS[@]}))
  echo "$(date +%H:%M) -----------------------> Started with run: $i crash_number: $crash_number"
  
  x=$(awk "BEGIN {print -1+2*$((RANDOM%=100))/100}")
  y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
  z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
  Y=$(awk "BEGIN {print 1.57-0.25+0.5*$((RANDOM%=100))/100}")
  LAUNCHFILE="${WORLDS[NUM]}.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y starting_height:=$z log_folder:=$LLOC"
  echo $COMMANDR
  START=$(date +%s)     
  if [ $DOCKER = true ] ; then 
    xterm -l -hold -e $COMMANDR &
  else
    xterm -hold -e $COMMANDR &
  fi
  pidlaunch=$!
  echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
  echo "Run started in xterm: $pidlaunch"
  while kill -0 $pidlaunch; 
  do 
    sleep 0.1
    END=$(date +%s)
    DIFF=$(( $END - $START ))
    if [ $DIFF -gt 300 ] ; 
    then
      if [ $crash_number -ge 3 ] ; then 
        echo "$(date +%H:%M) ########################### KILLED ROSCORE" >> "$LLOC/crash"       
        kill_combo
        crash_number=0
        echo "restart python:"
        if [ "$(ls $LLOC | wc -l)" -ge 5 ] ; then
          MODELDIR="$LOGDIR"
        else 
          rm -r $LLOC
        fi
        start_ros
        start_python
      else
        echo "$(date +%H:%M) #### KILLED ROSLAUNCH: $crash_number" >> "$LLOC/crash" 
        kill -9 $pidlaunch >/dev/null 2>&1
        sleep 2
        crash_number=$((crash_number+1))
        crashed=true
      fi
    fi
  done
  echo $(tail -1 ${LLOC}/log)
  if [[ $((i%5)) = 0 && $crashed = false ]] ; then
    python $HOME/catkin_ws/src/simulation_supervised/simulation_supervised_tools/python/viz_trajectories.py $LOGDIR True &
  fi  
  sleep 10
done
kill_combo





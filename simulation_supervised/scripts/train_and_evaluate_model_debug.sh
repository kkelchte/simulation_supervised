#!/bin/bash
######################################################
# Settings:
# $1=TAG
# $2=MODELDIR
# $3=NUMBER_OF_EPISODES
# $4=PARAMS
######################################################

if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
if [ -z $2 ] ; then
  MODELDIR="None"
else
  MODELDIR="$2"
fi
if [ -z $3 ] ; then
  NUMBER_OF_EPISODES=3
else
  NUMBER_OF_EPISODES="$3"
fi

PARAMS="${@:4}"
if [ $MODELDIR != "None" ] ; then
  # ensure continue training is true
  PARAMS="$(echo $PARAMS | sed 's/--continue_training False//' | sed 's/--continue_training True//') --continue_training True"
fi


TRAIN_NUMBER_OF_FLIGHTS=2
EVA_NUMBER_OF_FLIGHTS=2

TRAIN_WORLDS=(sandbox canyon forest)
# TRAIN_WORLDS=(sandbox esat_v2 canyon forest)
TRAIN_COUNTSUC=(0 0 0) 
TRAIN_COUNTTOT=(0 0 0)

EVA_WORLDS=(esat_v1 esat_v2 forest_real)
# EVA_WORLDS=(esat_v1 forest)
EVA_COUNTSUC=(0 0 0) 
EVA_COUNTTOT=(0 0 0)  

echo "TAG $TAG"
echo "MODELDIR $MODELDIR"
echo "NUMBER_OF_EPISODES $NUMBER_OF_EPISODES"
echo "TRAIN NUMBER OF FLIGHTS ${TRAIN_NUMBER_OF_FLIGHTS}"
echo "TRAIN WORLDS ${TRAIN_WORLDS[@]}"
echo "EVA NUMBER OF FLIGHTS ${EVA_NUMBER_OF_FLIGHTS}"
echo "EVA WORLDS ${EVA_WORLDS[@]}"

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
echo; echo;echo;
######################################################
# if folder already exists, continue training from there rather than restarting...
# if [ -e $HOME/tensorflow/log/$TAG ] ; then
#   rm -r $HOME/tensorflow/log/$TAG
#   # echo "ADJUSTED MODELDIR: continue training from $TAG instead of $MODELDIR"
#   # MODELDIR=$TAG
#   # # ensure continue training is true
#   # PARAMS="$(echo $PARAMS | sed 's/--continue_training False//' | sed 's/--continue_training True//') --continue_training True"
# fi   
mkdir -p $HOME/tensorflow/log/$TAG
cd $HOME/tensorflow/log/$TAG
if [ $docker = true ] ; then
  echo "RUNNING IN DOCKER"
  python_script="start_python_docker.sh"
else 
  echo "RUNNING LOCALLY"
  python_script="start_python.sh"
fi
LOGDIR="$TAG/$(date +%F_%H%M)"
LLOC="$HOME/tensorflow/log/$LOGDIR"
start_python(){
  ARGUMENTS="--log_tag $LOGDIR --checkpoint_path $MODELDIR $PARAMS"
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H%M%S) -hold -e $COMMANDP &
  pidpython=$!
  echo "PID Python tensorflow: $pidpython"
  sleep 35 #wait some seconds for model to load otherwise you miss the start message  
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
#location for logging
mkdir $LLOC/xterm_log

RUN_ROS(){
  crash_number=0
  echo "+++++++++++++++++++++++${RUN_TYPE}+++++++++++++++++++++"
  NUM=0
  i=$START_INDEX
  while [[ $i -lt $((START_INDEX+NUMBER_OF_FLIGHTS)) ]] ;
  do
    NUM=$((i%${#WORLDS[@]}))
    # If it is not esat/sandbox simulated, you can create a new world
    EXTRA_ARGUMENTS=""
    if [[ ${WORLDS[NUM]} == canyon  || ${WORLDS[NUM]} == forest ]] ; then
      python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $LLOC
      EXTRA_ARGUMENTS=" background:=$LLOC/${WORLDS[NUM]}.png world_name:=$LLOC/${WORLDS[NUM]}.world"
    fi
    if [ ${WORLDS[NUM]} == sandbox ] ; then
      SANDLOC=$(rospack find simulation_supervised_demo)/worlds/sandboxes_train
      EXTRA_ARGUMENTS=" background:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).png world_name:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).world"
    fi
    crashed=false
    # Clear gazebo log folder to overcome the impressive amount of log data
    if [[ $((i%50)) = 0 && i!=0 ]] ; then rm -r $HOME/.gazebo/log/* ; fi
    if [ ! -d $LLOC ] ; then echo "Cant locate $LLOC : esat is unmounted so stop." ; exit ; fi
    echo "$(date +%H:%M) -----------------------> Started with run: $i crash_number: $crash_number and world ${WORLDS[NUM]}"
    
    # x=$(awk "BEGIN {print -1+2*$((RANDOM%=100))/100}")
    # y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
    # z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
    # Y=$(awk "BEGIN {print 1.57-0.25+0.5*$((RANDOM%=100))/100}")
    speed=1.3
    x=0
    y=0
    if [ ${WORLDS[NUM]} == sandbox ] ; then
      z=0.5
    else 
      z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
    fi
    Y=1.57  
    LAUNCHFILE="${WORLDS[NUM]}.launch"
    COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
     Yspawned:=$Y x:=$x y:=$y starting_height:=$z speed:=$speed log_folder:=$LLOC\
     recovery:=$RECOVERY evaluate:=$evaluate $EXTRA_ARGUMENTS"
    echo $COMMANDR
    START_RUN=$(date +%s)     
    xterm -l -lf $LLOC/xterm_log/run_${i}_$(date +%F_%H%M%S)_$RUN_TYPE -hold -e $COMMANDR &
    pidlaunch=$!
    echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
    echo "Run started in xterm: $pidlaunch"
    while kill -0 $pidlaunch; 
    do 
      if [ docker = true ] ; then 
        # if the last check was too long ago you probably got suspended
        if [[ $(( $(date +%s) - START_RUN - DIFF)) -gt 30 ]] ; then
          sleep 30 #wait for big tick to update
          TOTAL_SUS="$(condor_q -glob -l $cluser_id | grep TotalSuspensions | tail -1 | cut -d ' ' -f 3)"
          echo "I was suspended for the $TOTAL_SUS 'th time."
          START_RUN=$(( $(date +%s) - $DIFF ))
        fi
      fi
      END=$(date +%s)
      DIFF=$(( $END - $START_RUN ))
      if [ $DIFF -gt 300 ] ; 
      then
        if [ $crash_number -ge 3 ] ; then 
          echo "$(date +%H:%M) ########################### KILLED ROSCORE" >> $LLOC/crash      
          echo "$(date +%H:%M) ########################### KILLED ROSCORE"     
          sleep 0.5
          kill_combo
          crash_number=0
          echo "restart python:"
          # if [ "$(ls $LLOC | wc -l)" -ge 6 ] ; then
          MODELDIR="$LLOC"
          PARAMS="$(echo $PARAMS | sed 's/--continue_training False//' | sed 's/--continue_training True//') --continue_training True"
          # else 
          #   rm -r $LLOC
          # fi
          start_ros
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
    done
    if [[ crashed != true ]] ; then
      i=$((i+1))
      if [ $(tail -1 $LLOC/log) == 'success' ] ; then
        COUNTSUC[NUM]="$((COUNTSUC[NUM]+1))"
      fi
      COUNTTOT[NUM]="$((COUNTTOT[NUM]+1))"
      echo "$(date +%F_%H-%M) finished ${RUN_TYPE}_run $i in world ${WORLDS[NUM]} with $(tail -1 ${LLOC}/log) resulting in ${COUNTSUC[NUM]} / ${COUNTTOT[NUM]}"
      sleep 5
    fi
    sleep 3
  done
}

START_INDEX=0
episode=0
while [ $episode -lt $NUMBER_OF_EPISODES ] ; do

  RUN_TYPE=EVAL
  START_INDEX=$((START_INDEX+TRAIN_NUMBER_OF_FLIGHTS))
  NUMBER_OF_FLIGHTS=$EVA_NUMBER_OF_FLIGHTS
  WORLDS=(${EVA_WORLDS[@]})
  COUNTSUC=(${EVA_COUNTSUC[@]})
  COUNTTOT=(${EVA_COUNTTOT[@]})
  evaluate=true
  RUN_ROS
  EVA_COUNTSUC=(${COUNTSUC[@]})
  EVA_COUNTTOT=(${COUNTTOT[@]})
  
  RUN_TYPE=TRAIN
  NUMBER_OF_FLIGHTS=$TRAIN_NUMBER_OF_FLIGHTS
  WORLDS=(${TRAIN_WORLDS[@]})
  COUNTSUC=(${TRAIN_COUNTSUC[@]})
  COUNTTOT=(${TRAIN_COUNTTOT[@]})
  evaluate=false
  RUN_ROS
  TRAIN_COUNTSUC=(${COUNTSUC[@]})
  TRAIN_COUNTTOT=(${COUNTTOT[@]})

  

  episode=$((episode+1))

done

kill_combo
echo 'done'
date +%F_%H%M%S

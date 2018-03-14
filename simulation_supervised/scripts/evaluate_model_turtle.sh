#!/bin/bash
######################################################
# Settings:
# -t TAG
# -m MODELDIR
# -n NUMBER_OF_FLIGHTS
# -w WORLDS
# -p PARAMS
######################################################

usage() { echo "Usage: $0 [-t LOGTAG: tag used to name logfolder]
    [-m MODELDIR: checkpoint to initialize weights with in logfolder]
    [-n NUMBER_OF_FLIGHTS]
    [-w \" WORLDS \" : space-separated list of environments ex \" canyon forest sandbox \"]
    [-s \" python_script \" : choose the python script to launch tensorflow: start_python or start_python_docker or start_python_sing_ql or start_python_sing_pg]
    [-p \" PARAMS \" : space-separated list of tensorflow flags ex \" --max_episodes 20 \" ]" 1>&2; exit 1; }
python_script="start_python_sing_ql.sh"
NUMBER_OF_FLIGHTS=2
TAG=test_evaluate_online
GRAPHICS=true
RECOVERY=false
while getopts ":t:m:n:w:s:p:g:r:" o; do
    case "${o}" in
        t)
            TAG=${OPTARG} ;;
        m)
            MODELDIR=${OPTARG} ;;
        n)
            NUMBER_OF_FLIGHTS=${OPTARG} ;;
        w)
            WORLDS+=(${OPTARG}) ;;
        s)
            python_script=${OPTARG} ;;
        p)
            PARAMS+=(${OPTARG}) ;;
        g)
            GRAPHICS=${OPTARG} ;; 
        r)
            RECOVERY=${OPTARG} ;; 
        *)
            usage ;;
    esac
done
shift $((OPTIND-1))

if [ -z "$WORLDS" ] ; then
  WORLDS=(canyon)
fi

if [ -z "$MODELDIR" ] ; then
  echo "$(tput setaf 1) (evaluate_model_turtle.sh): NO MODEL PROVIDED TO EVALUATE."
  tput sgr 0 
  exit
fi

echo "+++++++++++++++++++++++EVALUATE+++++++++++++++++++++"
echo "TAG=$TAG"
echo "MODELDIR=$MODELDIR"
echo "NUMBER_OF_FLIGHTS=$NUMBER_OF_FLIGHTS"
echo "WORLDS=${WORLDS[@]}"
echo "PYTHON SCRIPT=$python_script"
echo "PARAMS=${PARAMS[@]}"
echo "GRAPHICS=$GRAPHICS"
echo "RECOVERY=$RECOVERY"

RANDOM=125 #seed the random sequence
# Change params to string in order to parse it with sed.
PARAMS="${PARAMS[@]}"
PARAMS="$(echo $PARAMS | sed 's/--scratch\s//')"
# ensure continue_training is True [without assuming anything]
PARAMS="$(echo $PARAMS | sed 's/--continue_training\s//')"
PARAMS="$PARAMS --continue_training"

if [ -z $(echo $PARAMS | grep load_config) ] ; then
  PARAMS="$PARAMS --load_config"
fi
######################################################
# Start roscore and load general parameters
start_ros(){
  echo "start_ros"
  roslaunch simulation_supervised load_params.launch global_param:=turtle_param.yaml drone_config:=sim_turtle.yaml&
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 10  
}
start_ros

######################################################
# If graphics is false ensure showdepth is false
if [ $GRAPHICS = false ] ; then
  PARAMS="$(echo $PARAMS | sed 's/--show_depth\s//')"
  PARAMS="$PARAMS --show_depth"
fi  
######################################################
# Start tensorflow with command defined above
mkdir -p $HOME/tensorflow/log/$TAG
cd $HOME/tensorflow/log/$TAG

start_python(){
  echo "start python"
  LOGDIR="$TAG/$(date +%F_%H%M)_eval"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  ARGUMENTS="--log_tag $LOGDIR --checkpoint_path $MODELDIR $PARAMS"
  if [ $RECOVERY = true ] ; then
    ARGUMENTS="$ARGUMENTS --recovery"
  fi
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H-%M) -hold -e $COMMANDP &
  pidpython=$!
  echo "PID Python tensorflow: $pidpython"
  cnt=0
  while [ ! -e $LLOC/tf_log ] ; do 
    sleep 1 
    cnt=$((cnt+1))
    if [ $cnt -gt 600 ] ; then 
      echo "$(tput setaf 1) Waited for 5minutes on tf_log, seems like tensorlfow crashed... on $(cat $_CONDOR_JOB_AD | grep RemoteHost | head -1 | cut -d '=' -f 2 | cut -d '@' -f 2 | cut -d '.' -f 1) $(tput sgr 0)"
      echo "$(tput setaf 1) Waited for 5minutes on tf_log, seems like tensorlfow crashed... on $(cat $_CONDOR_JOB_AD | grep RemoteHost | head -1 | cut -d '=' -f 2 | cut -d '@' -f 2 | cut -d '.' -f 1) $(tput sgr 0)" > /esat/opal/kkelchte/docker_home/.debug/$TAG
      restart
    fi 
  done
}
start_python

# create location for logging the xterm outputs.
XLOC=$HOME/tensorflow/log/${TAG}/xterm_log  
mkdir -p $XLOC
######################################################
# kill all processes
kill_combo(){
  echo "kill ros:"
  kill -9 $pidlaunch > /dev/null 2>&1 
  killall -9 roscore > /dev/null 2>&1 
  killall -9 rosmaster > /dev/null 2>&1
  killall -9 /*rosout* > /dev/null 2>&1 
  killall -9 gzclient > /dev/null 2>&1
  kill -9 $pidros > /dev/null 2>&1
  for i in $(ps -ef | grep ros | grep -v grep | cut -d ' ' -f 2) ; do 
    while kill -0 $i >/dev/null 2>&1 ; do 
      kill $i 2>&1 > /dev/null 
      sleep 0.5
    done
  done
  while kill -0 $pidpython > /dev/null 2>&1 ;
  do      
    kill $pidpython >/dev/null 2>&1
    sleep 0.05
  done
  sleep 5
}
######################################################
# restart ros-python-ros
restart(){
  kill_combo
  crash_number=0
  #location for logging
  start_ros
  start_python
}

crash_number=0

flight_num=0
while [[ $flight_num -lt $NUMBER_OF_FLIGHTS ]] ;
do
  echo "run: $flight_num"
  NUM=$((flight_num%${#WORLDS[@]}))

  if [ -e $LLOC/tf_log ] ; then
    old_stat="$(stat -c %Y $LLOC/tf_log)"
  else
    echo "Could not find $LLOC/tf_log"
  fi

  # If it is not esat simulated, you can create a new world
  EXTRA_ARGUMENTS=""
  if [[ ${WORLDS[NUM]} == canyon  || ${WORLDS[NUM]} == forest || ${WORLDS[NUM]} == sandbox ]] ; then
    python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $LLOC
    EXTRA_ARGUMENTS=" background:=$LLOC/${WORLDS[NUM]}.png world_name:=$LLOC/${WORLDS[NUM]}.world"
  fi
  crashed=false
  # Clear gazebo log folder to overcome the impressive amount of log data
  if [ $((flight_num%50)) = 0 ] ; then rm -r $HOME/.gazebo/log/* ; fi
  if [[ ! -d $LLOC ]] ; then echo "$(tput setaf 1)log location is unmounted so stop.$(tput sgr 0)" ; kill_combo; exit ; fi
  echo "$(date +%F_%H-%M) -----------------------> Started with run: $flight_num crash_number: $crash_number"
  x=0
  y=0
  Y=1.57
  LAUNCHFILE="${WORLDS[NUM]}_turtle.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y log_folder:=$LLOC evaluate:=true\
   $EXTRA_ARGUMENTS graphics:=$GRAPHICS"
  echo $COMMANDR
  # STARTING TIME
  START=$(date +%s)
  # TIME SPAN TRAINING/EVALUATING     
  TS=0
  # CURRENT TIME
  NOW=$(date +%s)

  xterm -iconic -l -lf $XLOC/run_${flight_num}_$(date +%F_%H-%M) -hold -e $COMMANDR &
  pidlaunch=$!
  echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
  echo "Run started in xterm: $pidlaunch"
  while kill -0 $pidlaunch > /dev/null 2>&1; 
  do 
    NOW=$(date +%s)
    # Check if job got suspended: if between last update and now has been more than 30 seconds (should be less than 0.1s)
    if [[ $(( NOW - START - TS)) -gt 30 ]] ; then
      sleep 30 #wait for big tick to update
      TOTAL_SUS="$(condor_q -glob -l $cluster_id | grep TotalSuspensions | tail -1 | cut -d ' ' -f 3)"
      echo "I was suspended for the $TOTAL_SUS 'th time."
      START=$(( NOW - TS ))
    else
      # otherwise: time span update: 
      TS=$(( NOW - START ))
    fi
    
    if [ $TS -gt 300 ] ; 
    then
      echo "$(tput setaf 1) ---CRASH (delay time: $TS) $(tput sgr 0)"
      if [ $crash_number -ge 3 ] ; then
        message="$(date +%F_%H-%M) ########################### KILLED ROSCORE" 
        echo $message >> $LLOC/crash      
        echo $message     
        sleep 0.5
        restart
        crashed=true
      else
        message="$(date +%F_%H-%M) #### KILLED ROSLAUNCH: $crash_number"
        echo $message >> $LLOC/crash
        echo $message
        sleep 0.5
        kill -9 $pidlaunch >/dev/null 2>&1
        sleep 2
        crash_number=$((crash_number+1))
        crashed=true
      fi
    fi
    sleep 0.1
  done
  if [ $crashed != true ] ; then
    # wait for tensorflow
    if [ -e $LLOC/tf_log ] ; then 
      cnt=0
      new_stat="$(stat -c %Y $LLOC/tf_log)"
      while [ $old_stat = $new_stat ] ; do 
        new_stat="$(stat -c %Y $LLOC/tf_log)"
        cnt=$((cnt+1)) 
        if [ $cnt -gt 300 ] ; then 
          echo "[evaluate_model.sh] Waited for 5minutes on tf_log restarting python and ROS."
          restart
        fi 
        sleep 1
      done
    else 
      echo "[evaluate_model.sh] Could not find $LLOC/tf_log"
      exit 
    fi
    flight_num=$((flight_num+1))
    if [ $(tail -1 $LLOC/log) == 'success' ] ; then
      COUNTSUC[NUM]="$((COUNTSUC[NUM]+1))"
    fi
    COUNTTOT[NUM]="$((COUNTTOT[NUM]+1))"
    echo "$(date +%F_%H-%M) finished run $flight_num in world ${WORLDS[NUM]} with $(tail -1 ${LLOC}/log) resulting in ${COUNTSUC[NUM]} / ${COUNTTOT[NUM]}"
  fi  
done
kill_combo
date +%F_%H-%M
echo 'done'

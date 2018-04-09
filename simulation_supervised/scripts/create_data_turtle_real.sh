#!/bin/bash
######################################################
# Settings:
# -t TAG
# -m MODELDIR
# -n NUMBER_OF_FLIGHTS
# -w WORLDS
# -p PARAMS
######################################################

usage() { echo "Usage: $0 [-t LOGTAG: tag used to name datafolder]
    [-m MODELDIR: checkpoint to initialize weights within logfolder]
    [-n NUMBER_OF_FLIGHTS]
    [-w \" WORLDS \" : space-separated list of environments ex \" canyon forest sandbox \"]
    [-s \" python_script \" : choose the python script to launch tensorflow: start_python or start_python_docker or start_python_sing]
    [-p \" PARAMS \" : space-separated list of tensorflow flags ex \" --auxiliary_depth True --continue_training False\" ]" 1>&2; exit 1; }
python_script="start_python.sh"
NUMBER_OF_FLIGHTS=1
TAG=test_createdata
GRAPHICS=true
NOISE=ou
MODELDIR=""
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
            NOISE=${OPTARG} ;; 
        *)
            usage ;;
    esac
done
shift $((OPTIND-1))

if [ -z "$WORLDS" ] ; then
  WORLDS=(canyon)
fi

echo "+++++++++++++++++++++++CREATE DATA+++++++++++++++++++++"
echo "TAG=$TAG"
echo "MODELDIR=$MODELDIR"
echo "NUMBER_OF_FLIGHTS=$NUMBER_OF_FLIGHTS"
echo "WORLDS=${WORLDS[@]}"
echo "PYTHON SCRIPT=$python_script"
echo "PARAMS=${PARAMS[@]}"
echo "GRAPHICS=$GRAPHICS"
echo "NOISE=$NOISE"

export ROS_MASTER_URI=http://10.42.0.1:11311 && export ROS_HOSTNAME=10.42.0.203

RANDOM=125 #seed the random sequence
# Change params to string in order to parse it with sed.
PARAMS="${PARAMS[@]}"

if [ -d $HOME/pilot_data/$TAG ] ; then
  run_num="$(ls $HOME/pilot_data/$TAG | grep 0 | wc -l)"
  echo "found $TAG already and will continue from run: $run_num"
fi
DATA_LLOC="$HOME/pilot_data/$TAG"
mkdir -p $DATA_LLOC/xterm_log

######################################################
# Start tensorflow with command defined above if model is provided for flying
# make rosparam supervision True so BA's control is set to /supervised_vel


start_python(){
  echo "start python"
  LOGDIR="$TAG/$(date +%F_%H%M)_create_data"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  ARGUMENTS="--log_tag $LOGDIR $PARAMS --noise $NOISE"
  if [ ! -z $MODELDIR ] ; then
    ARGUMENTS="$ARGUMENTS --checkpoint_path $MODELDIR"
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
      kill_combo
      restart
    fi 
  done
}
start_python


######################################################
# kill all processes
kill_combo(){
  echo "kill ros:"
  kill -9 $pidlaunch > /dev/null 2>&1 
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
  start_python
}

crash_number=0

run_num=0
while [[ $run_num -lt $NUMBER_OF_FLIGHTS ]] ;
do
  echo "run: $run_num"
  NUM=$((run_num%${#WORLDS[@]}))

  if [ -e $LLOC/tf_log ] ; then
    old_stat="$(stat -c %Y $LLOC/tf_log)"
  else
    echo "Could not find $LLOC/tf_log"
  fi
  # If it is not esat simulated, you can create a new world
  EXTRA_ARGUMENTS=""
  if [[ ${WORLDS[NUM]} == canyon  || ${WORLDS[NUM]} == forest || ${WORLDS[NUM]} == sandbox ]] ; then
    python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $DATA_LLOC
    EXTRA_ARGUMENTS=" background:=$DATA_LLOC/${WORLDS[NUM]}.png world_name:=$DATA_LLOC/${WORLDS[NUM]}.world"
  fi
  crashed=false
  # Clear gazebo log folder to overcome the impressive amount of log data
  if [ $((i%50)) = 0 ] ; then rm -r $HOME/.gazebo/log/* ; fi
  if [[ ! -d $DATA_LLOC ]] ; then echo "$(tput setaf 1)log location is unmounted so stop.$(tput sgr 0)" ; kill_combo; exit ; fi
  echo "$(date +%H:%M) -----------------------> Started with run: $i crash_number: $crash_number"
  x=$(awk "BEGIN {print -0.5+$((RANDOM%=100))/100}")
  y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
  if [[ ${WORLDS[NUM]} = sandbox ]] ; then
    z=0.5
    Y=$(awk "BEGIN {print 1.57-0.5+$((RANDOM%=100))/100}")
  else
    Y=$(awk "BEGIN {print 1.57-0.15+0.3*$((RANDOM%=100))/100}")
  fi
  saving_location=$DATA_LLOC/$(printf %05d $i)_${WORLDS[NUM]}
  
  LAUNCHFILE="${WORLDS[NUM]}_turtle.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y log_folder:=$DATA_LLOC\
   saving_location:=$saving_location evaluate:=true noise:=$NOISE recovery:=$RECOVERY\
   $EXTRA_ARGUMENTS graphics:=$GRAPHICS"
  echo $COMMANDR
  # STARTING TIME
  START=$(date +%s)
  # TIME SPAN TRAINING/EVALUATING     
  TS=0
  # CURRENT TIME
  NOW=$(date +%s)

  xterm -iconic -l -lf $DATA_LLOC/xterm_log/run_${i}_$(date +%F_%H-%M) -hold -e $COMMANDR &
  pidlaunch=$!
  echo $pidlaunch > $DATA_LLOC/$(rosparam get /pidfile)
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
        echo $message >> $DATA_LLOC/crash      
        echo $message     
        sleep 0.5
        restart
        crashed=true
      else
        message="$(date +%F_%H-%M) #### KILLED ROSLAUNCH: $crash_number"
        echo $message >> $DATA_LLOC/crash
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
          echo "[create_data_turtle.sh] Waited for 5minutes on tf_log restarting python and ROS."
          restart
        fi 
        sleep 1
      done
    else 
      echo "[train_model.sh] Could not find $LLOC/tf_log"
      exit 
    fi
    i=$((i+1))
    if [ $(tail -1 $DATA_LLOC/log) == 'success' ] ; then
      COUNTSUC[NUM]="$((COUNTSUC[NUM]+1))"
    fi
    COUNTTOT[NUM]="$((COUNTTOT[NUM]+1))"
    echo "$(date +%F_%H-%M) finished run $i in world ${WORLDS[NUM]} with $(tail -1 ${DATA_LLOC}/log) resulting in ${COUNTSUC[NUM]} / ${COUNTTOT[NUM]}"
    # keep the validation world, just in case...
    mv $LLOC/${WORLDS[NUM]}.world $saving_location  
  else #if it was a fail: clean up!
    rm -r $saving_location
  fi
  sleep 5
done
kill_combo
date +%F_%H-%M
echo 'done'

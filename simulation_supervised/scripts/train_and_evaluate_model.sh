#!/bin/bash
######################################################
# Train model online in the doshico training environments
# and evaluate the same model online in the ESAT environment
# Settings:
# -t TAG
# -m MODELDIR
# -n NUMBER_OF_FLIGHTS
# -w WORLDS
# -p PARAMS
######################################################

usage() { echo "Usage: $0 [-t LOGTAG: tag used to name logfolder]
    [-m MODELDIR: checkpoint to initialize weights with in logfolder]
    [-n NUMBER_OF_EPISODES]
    [-w \" WORLDS \" : space-separated list of environments ex \" canyon forest sandbox \"]
    [-s \" python_script \" : choose the python script to launch tensorflow: start_python or start_python_docker]
    [-p \" PARAMS \" : space-separated list of tensorflow flags ex \" --auxiliary_depth True\" ]" 1>&2; exit 1; }
python_script="start_python_docker.sh"
NUMBER_OF_FLIGHTS=2
GRAPHICS=true
while getopts ":t:m:n:p:w:s:" o; do
    case "${o}" in
        t)
            TAG=${OPTARG} ;;
        m)
            MODELDIR=${OPTARG} ;;
        n)
            NUMBER_OF_EPISODES=${OPTARG} ;;
        w)
            WORLDS+=(${OPTARG}) ;;
        s)
            python_script=${OPTARG} ;;
        p)
            PARAMS+=(${OPTARG}) ;;
        g)
            GRAPHICS=${OPTARG} ;; 
        *)
            usage ;;
    esac
done
shift $((OPTIND-1))

if [ -z "$WORLDS" ] ; then
	TRAIN_WORLDS=(canyon forest sandbox)
	EVA_WORLDS=(esat_v1 esat_v2)
else
	TRAIN_WORLDS=${WORLDS[@]}
	EVA_WORLDS=${WORLDS[@]}
fi
if [ $NUMBER_OF_EPISODES = 0 ] ; then 
  echo "debugging so only small set of runs."
  TRAIN_NUMBER_OF_FLIGHTS=4
  EVA_NUMBER_OF_FLIGHTS=1
  NUMBER_OF_EPISODES=3
else
  TRAIN_NUMBER_OF_FLIGHTS=40
  EVA_NUMBER_OF_FLIGHTS=10
fi

echo "+++++++++++++++++++++++TRAIN AND EVALUATE+++++++++++++++++++++"
echo "TAG=$TAG"
echo "MODELDIR=$MODELDIR"
echo "NUMBER_OF_FLIGHTS=$NUMBER_OF_FLIGHTS"
echo "TRAIN_WORLDS=${TRAIN_WORLDS[@]}"
echo "EVA_WORLDS=${EVA_WORLDS[@]}"
echo "PARAMS=${PARAMS[@]}"
echo "NUMBER_OF_EPISODES=$NUMBER_OF_EPISODES"
echo "TRAIN NUMBER OF FLIGHTS ${TRAIN_NUMBER_OF_FLIGHTS}"
echo "EVA NUMBER OF FLIGHTS ${EVA_NUMBER_OF_FLIGHTS}"
echo "GRAPHICS=$GRAPHICS"

RANDOM=125 #seed the random sequence

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
# if log folder already exists, continue training from there rather than restarting...
if [[ -e $HOME/tensorflow/log/$TAG && -e $HOME/tensorflow/log/$TAG/$(ls -t $HOME/tensorflow/log/$TAG | head -2 | grep -v xterm)/checkpoint ]] ; then
  echo "ADJUSTED MODELDIR: continue training from $TAG instead of $MODELDIR"
  MODELDIR=$TAG
  # ensure continue training is true
  PARAMS="$(echo $PARAMS | sed 's/--continue_training False//' | sed 's/--continue_training True//') --continue_training True"
fi   
mkdir -p $HOME/tensorflow/log/$TAG
cd $HOME/tensorflow/log/$TAG
start_python(){
  LOGDIR="$TAG/$(date +%F_%H%M)"
  LLOC="$HOME/tensorflow/log/$LOGDIR"
  ARGUMENTS="--log_tag $LOGDIR ${PARAMS[@]}"
  if [ ! -z $MODELDIR ] ; then
    ARGUMENTS="$ARGUMENTS --checkpoint_path $MODELDIR"
  fi
  COMMANDP="$(rospack find simulation_supervised)/scripts/$python_script $ARGUMENTS"
  echo $COMMANDP
  xterm -l -lf $HOME/tensorflow/log/$TAG/xterm_python_$(date +%F_%H%M%S) -hold -e $COMMANDP &
  pidpython=$!
  echo "PID Python tensorflow: $pidpython"
  while [ ! -e $LLOC/tf_log ] ; do 
    sleep 1 
    cnt=$((cnt+1)) 
    if [ $cnt -gt 300 ] ; then 
      echo "$(tput setaf 1) Waited for 5minutes on tf_log... $(tput sgr 0)" 
      exit 
    fi 
  done
  # sleep 20 #wait some seconds for model to load otherwise you miss the start message  
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
mkdir -p $LLOC/xterm_log

RUN_ROS(){
  crash_number=0
  echo "+++++++++++++++++++++++${RUN_TYPE}+++++++++++++++++++++"
  NUM=0
  i=$START_INDEX
  while [[ $i -lt $((START_INDEX+NUMBER_OF_FLIGHTS)) ]] ;
  do
    NUM=$((i%${#WORLDS[@]}))
    if [ -e $LLOC/tf_log ] ; then
      old_stat="$(stat -c %Y $LLOC/tf_log)"
    else
      echo "Could not find $LLOC/tf_log"
    fi
    # If it is not esat/sandbox simulated, you can create a new world
    EXTRA_ARGUMENTS=""
    if [[ ${WORLDS[NUM]} == canyon  || ${WORLDS[NUM]} == forest || ${WORLDS[NUM]} == sandbox ]] ; then
      python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $LLOC
      EXTRA_ARGUMENTS=" background:=$LLOC/${WORLDS[NUM]}.png world_name:=$LLOC/${WORLDS[NUM]}.world"
    fi
    crashed=false
    # Clear gazebo log folder to overcome the impressive amount of log data
    if [[ $((i%50)) = 0 && i!=0 ]] ; then rm -r $HOME/.gazebo/log/* ; fi
    if [[ ! -d $LLOC ]] ; then echo "$(tput setaf 1)log location is unmounted so stop.$(tput sgr 0)" ; kill_combo; exit ; fi
    echo "$(date +%H:%M) -----------------------> Started with run: $i crash_number: $crash_number and world ${WORLDS[NUM]}"
    x=0
    Y=1.57  
    if [ $evaluate = true ] ; then
      y=$(awk "BEGIN {print -0.25+0.5*$((RANDOM%=100))/100}")   
      if [ ${WORLDS[NUM]} == sandbox ] ; then
        z=$(awk "BEGIN {print 0.4+0.2*$((RANDOM%=100))/100}")
      else 
        z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
      fi
    else
      y=0 
      if [ ${WORLDS[NUM]} == sandbox ] ; then
        z=0.5
      else 
        z=1
      fi
    fi
    LAUNCHFILE="${WORLDS[NUM]}.launch"
    COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
     Yspawned:=$Y x:=$x y:=$y starting_height:=$z log_folder:=$LLOC\
     recovery:=$RECOVERY evaluate:=$evaluate $EXTRA_ARGUMENTS"
    echo $COMMANDR
    START=$(date +%s)     
    xterm -l -lf $LLOC/xterm_log/run_${i}_$(date +%F_%H%M%S)_$RUN_TYPE -hold -e $COMMANDR &
    pidlaunch=$!
    echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
    echo "Run started in xterm: $pidlaunch"
    while kill -0 $pidlaunch; 
    do 
      END=$(date +%s)
      DIFF=$(( $END - $START ))
      if [ $DIFF -gt 300 ] ; 
      then
        echo "$(tput setaf 1) ---CRASH (delay time: $DIFF) $(tput sgr 0)"
        if [ $crash_number -ge 3 ] ; then 
          message="$(date +%H:%M) ########################### KILLED ROSCORE" 
          echo $message >> $LLOC/crash      
          echo $message     
          sleep 0.5
          kill_combo
          crash_number=0
          #location for logging
          mkdir -p $LLOC/xterm_log
          echo "restart python:"
          MODELDIR="$LLOC"
          PARAMS="$(echo $PARAMS | sed 's/--continue_training False//' | sed 's/--continue_training True//') --continue_training True"
          start_ros
          start_python
        else
          message="$(date +%H:%M) #### KILLED ROSLAUNCH: $crash_number"
          echo $message >> $LLOC/crash
          echo $message
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
      # wait for tensorflow
    if [ -e $LLOC/tf_log ] ; then 
      new_stat="$(stat -c %Y $LLOC/tf_log)"
      while [ $old_stat = $new_stat ] ; do 
        new_stat="$(stat -c %Y $LLOC/tf_log)"
        sleep 1
      done
    else 
      cnt=0
      while [ ! -e $LLOC/tf_log ] ; do 
        sleep 1 
        cnt=$((cnt+1)) 
        if [ $cnt -gt 300 ] ; then 
          echo "Waited for 5minutes on tf_log..." 
          exit 
        fi 
      done
    fi
  done
}

START_INDEX=0
episode=0
while [ $episode -lt $NUMBER_OF_EPISODES ] ; do
  RUN_TYPE=TRAIN
  NUMBER_OF_FLIGHTS=$TRAIN_NUMBER_OF_FLIGHTS
  WORLDS=(${TRAIN_WORLDS[@]})
  COUNTSUC=(${TRAIN_COUNTSUC[@]})
  COUNTTOT=(${TRAIN_COUNTTOT[@]})
  evaluate=false
  RUN_ROS
  TRAIN_COUNTSUC=(${COUNTSUC[@]})
  TRAIN_COUNTTOT=(${COUNTTOT[@]})

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

  episode=$((episode+1))

done
kill_combo
date +%F_%H%M%S
echo 'done'

#!/bin/bash
######################################################
# Settings:
# -t TAG
# -m MODELDIR
# -w WORLD
# -p PARAMS
######################################################

usage() { echo "Usage: $0 [-t LOGTAG: tag used to name logfolder]
    [-m MODEL: checkpoint to initialize weights with in logfolder]
    [-w WORLD: the environment you want to fly in]
    [-p \" PARAMS \" : space-separated list of tensorflow flags ex \" --auxiliary_depth True --max_episodes 20 \" ]" 1>&2; exit 1; }

while getopts ":t:m:n:p:w:" o; do
    case "${o}" in
        t)
            TAG=${OPTARG}
            ;;
        m)
            MODEL=${OPTARG}
            ;;
        w)
            WORLD=${OPTARG}
            ;;
        p)
            PARAMS="${OPTARG}"
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

echo "+++++++++++++++++++++++FLY IN SIMULATION+++++++++++++++++++++"
echo "TAG $TAG"
echo "MODELDIR $MODELDIR"
echo "WORLD ${WORLD[@]}"
echo "PARAMS=${PARAMS[@]}"


RANDOM=125 #seed the random sequence

######################################################
# Start roscore and load general parameters
start_ros(){
  echo "start ROS"
  roslaunch simulation_supervised load_params.launch global_param:=online_param.yaml&
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 3  
}
start_ros

######################################################
mkdir -p $HOME/tensorflow/log/$TAG
cd $HOME/tensorflow/log/$TAG
python_script="start_python.sh"
LOGDIR="$TAG/$(date +%F_%H%M)_fly"
LLOC="$HOME/tensorflow/log/$LOGDIR"
start_python(){
  ARGUMENTS="--off_policy True --show_depth True --show_odom True --continue_training True --evaluate True --log_tag $LOGDIR --checkpoint_path $MODELDIR --load_config True"
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
finished=false
while [ $finished != true ] ; do
  EXTRA_ARGUMENTS=""
  if [[ ${WORLD} == forest || ${WORLD} == canyon || ${WORLD} == sandbox]] ; then
    python $(rospack find simulation_supervised_tools)/python/${WORLD}_generator.py $LLOC
    EXTRA_ARGUMENTS=" background:=$LLOC/${WORLD}.png world_name:=$LLOC/${WORLD}.world"
  fi
    
  # x=$(awk "BEGIN {print -1+2*$((RANDOM%=100))/100}")
  # y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
  # z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
  # Y=$(awk "BEGIN {print 1.57-0.25+0.5*$((RANDOM%=100))/100}")

  speed=1.3
  x=0
  y=0
  Y=1.57  

  LAUNCHFILE="${WORLD}_joy.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y starting_height:=-1 speed:=$speed log_folder:=$LLOC\
   docker:=$docker $EXTRA_ARGUMENTS"
  xterm -l -lf $LLOC/xterm_log/run_${i}_$(date +%F_%H%M%S) -hold -e $COMMANDR &
  pidlaunch=$!
  echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
  echo "Run started in xterm: $pidlaunch"
  while kill -0 $pidlaunch; 
  do 
    sleep 0.1
  done

  echo
  echo ''''''''''''''''''''''''''''''''''''''''''''''
  tail -1 $LLOC/log
  echo ''''''''''''''''''''''''''''''''''''''''''''''


  read -p 'you want more? [y]/n: ' answer
  if [ -z $answer ] ; then
  read -p 'next world? (sandbox, canyon, forest, esat_v1, esat_v2) ' WORLD
  else
    if [ $answer == 'n' ] ; then
      echo 'finished then.'
      finished=true
    else
      read -p 'next world? (sandbox, canyon, forest, esat_v1, esat_v2) ' WORLD
    fi

  fi
done
kill_combo
echo 'done'
date +%F_%H%M%S





#!/bin/bash

######################################################
# $1=NAME
######################################################

NAME="location"
if [ ! -z $1 ] ; then
  NAME="$1"
fi

RANDOM=125 #seed the random sequence

######################################################
# Start roscore and load general parameters
start_ros(){
  echo "start ROS"
  COMMAND="roslaunch simulation_supervised load_params.launch global_param:=data_param.yaml"
  xterm -hold -e $COMMAND &
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 10  
}
start_ros
######################################################
mkdir -p $HOME/pilot_data/$NAME/xterm_log

######################################################
# Start ros with launch file
kill_combo(){
  echo "kill ros:"
  kill -9 $pidlaunch >/dev/null 2>&1 
  killall -9 roscore >/dev/null 2>&1 
  killall -9 rosmaster >/dev/null 2>&1
  killall -9 /*rosout* >/dev/null 2>&1 
  killall -9 gzclient >/dev/null 2>&1
  kill -9 $pidros >/dev/null 2>&1
  sleep 10
}

crash_number=0
start_time=$(date +%s)
#location for logging
LLOC="$HOME/pilot_data/$NAME"
finished=false
LAUNCHFILE="bebop_real_data.launch"
COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE"
echo $COMMANDR
START=$(date +%s)     
xterm -l -lf $LLOC/xterm_log/run_${i}_$(date +%F_%H%M%S) -hold -e $COMMANDR &
pidlaunch=$!
echo $pidlaunch > $LLOC/$(rosparam get /pidfile)
echo "Run started in xterm: $pidlaunch"

######################################################
finished=false
i=0
while [ $finished != true ] ; do
  read -p 'camera direction? r, s,l ' d
  if [ $d == 'r' ] ; then
    echo 'right'
    direction='right'
  fi
  if [ $d == 's' ] ; then
    echo 'straight'
    direction='straight'
  fi
  if [ $d == 'l' ] ; then
    echo 'left'
    direction='left'
  fi
  read -p 'type of collision: perspective, vertical, strange: p,v,s ' t
  if [ $t == 'p' ] ; then
    echo 'perspective'
    coltype='perspective'
  fi
  if [ $t == 'v' ] ; then
    echo 'vertical'
    coltype='vertical'
  fi
  if [ $t == 's' ] ; then
    echo 'strange'
    coltype='strange'
  fi
  saving_location="$LLOC/$(printf %05d $i)_${coltype}_${direction}"
  xterm -hold -e roslaunch create_dataset create_data.launch saving_location:=$saving_location direction:=$direction

  read -p 'you want more? [y]/n: ' answer
  if [ $answer == 'n' ] ; then
    echo 'finished then.'
    finished=true
  else
    NEWNAME=''
    read -p 'NAME? ' NEWNAME
    if [ -z $NEWNAME ] ; then
      i=$((i+1))    
    else
      LLOC="$HOME/pilot_data/$NEWNAME"
      echo "new saving location: $NEWNAME"
      i=0
    fi
  fi
done  

kill_combo
echo 'done'
date +%F_%H%M%S





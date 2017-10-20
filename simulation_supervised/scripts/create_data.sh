#!/bin/bash

######################################################
# $1=NAME
# $2=WORLD
# $3=NUMBER_OF_FLIGHTS
######################################################

NAME="dataset"
if [ ! -z $1 ] ; then
  NAME="$1"
fi
WORLDS=(esat_v1 esat_v2) 
COUNTSUC=(0 0)
COUNTTOT=(0 0)  
if [ ! -z $2 ] ; then
  WORLDS=($2)
  COUNTSUC=(0)
  COUNTTOT=(0)
fi
if [ -z $3 ] ; then
  NUMBER_OF_FLIGHTS=105
else
  NUMBER_OF_FLIGHTS="$3"
fi
NOISE="ou" #ou uni none 
echo "NAME $NAME"
echo "WORLDS ${WORLDS[@]}"
echo "NOISE $NOISE"
echo "NUMBER OF FLIGHTS ${NUMBER_OF_FLIGHTS}"

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
  roslaunch simulation_supervised load_params.launch global_param:=data_param.yaml&
  pidros=$!
  echo "PID ROS: " $pidros
  sleep 10  
}
start_ros
######################################################
# DELETE DIRECTORY IF IT ALREADY EXISTS
if [ -d $HOME/pilot_data/$NAME ] ; then
  rm -r $HOME/pilot_data/$NAME
fi
mkdir -p $HOME/pilot_data/$NAME/xterm_log

######################################################
convertsecs() {
  ((h=${1}/3600))
  ((m=(${1}%3600)/60))
  ((s=${1}%60))
  printf "%02d hours %02d min %02d sec\n" $h $m $s
}
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
i=0
while [[ $i -lt $NUMBER_OF_FLIGHTS ]] ;
do
  NUM=$((i%${#WORLDS[@]}))
  # If it is not esat simulated, you can create a new world
  EXTRA_ARGUMENTS=""
  if [[ ${WORLDS[NUM]} == forest && ${WORLDS[NUM]} == canyon ]] ; then
    python $(rospack find simulation_supervised_tools)/python/${WORLDS[NUM]}_generator.py $LLOC
    EXTRA_ARGUMENTS=" background:=$LLOC/${WORLDS[NUM]}.png world_name:=$LLOC/${WORLDS[NUM]}.world"
  fi
  if [ ${WORLDS[NUM]} == sandbox ] ; then
    SANDLOC=$(rospack find simulation_supervised_demo)/worlds/sandboxes_offline
    EXTRA_ARGUMENTS=" background:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).png world_name:=$SANDLOC/${WORLDS[NUM]}_$(printf %05d $i).world"
  fi
  crashed=false
  # Clear gazebo log folder to overcome the impressive amount of log data
  if [ $((i%50)) = 0 ] ; then rm -r $HOME/.gazebo/log/* ; fi
  if [ ! -d $LLOC ] ; then echo 'esat is unmounted so stop.' ; exit ; fi
  # launch world
  x=$(awk "BEGIN {print -0.5+$((RANDOM%=100))/100}")
  y=$(awk "BEGIN {print $((RANDOM%=100))/100}")   
  if [[ ${WORLDS[NUM]} = sandbox ]] ; then
    z=0.5
    Y=$(awk "BEGIN {print 1.57-0.5+$((RANDOM%=100))/100}")
  else
    z=$(awk "BEGIN {print 0.5+1.*$((RANDOM%=100))/100}")
    Y=$(awk "BEGIN {print 1.57-0.15+0.3*$((RANDOM%=100))/100}")
  fi
  speed=$(awk "BEGIN {print 1.3-0.5+$((RANDOM%=100))/100}") #vary from 0.8 till 1.8
  # speed=1.3
  # x=+0.5
  # Y=1.42
  # x=-0.5
  # Y=1.57
  saving_location=$LLOC/$(printf %05d $i)_${WORLDS[NUM]}
  LAUNCHFILE="${WORLDS[NUM]}.launch"
  COMMANDR="roslaunch simulation_supervised_demo $LAUNCHFILE\
   Yspawned:=$Y x:=$x y:=$y starting_height:=$z speed:=$speed log_folder:=$LLOC\
   saving_location:=$saving_location noise:=$NOISE recovery:=$RECOVERY $EXTRA_ARGUMENTS"
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
        start_ros
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
  if [ crashed != true ] ; then 
    if [ $(tail -1 ${LLOC}/log) == 'success' ] ; then
      COUNTSUC[NUM]="$((COUNTSUC[NUM]+1))"
    fi
    COUNTTOT[NUM]="$((COUNTTOT[NUM]+1))"
    echo "$(date +%F_%H-%M) finished run $i in world ${WORLDS[NUM]} with $(tail -1 ${LLOC}/log) resulting in ${COUNTSUC[NUM]} / ${COUNTTOT[NUM]}"
  fi
  if [ $(tail -1 ${LLOC}/log) == 'success' ] ; then
    if [ $(ls -l $saving_location/RGB | wc -l) -gt 20 ] ; then
      if [ crashed != true ] ; then
        i=$((i+1))
      else #if it was a fail: clean up!
        rm -r $saving_location
      fi
    else #if it was a fail: clean up!
      rm -r $saving_location
    fi
  else #if it was a fail: clean up!
    rm -r $saving_location
  fi
  
done
kill_combo
echo 'done'
date +%F_%H%M%S





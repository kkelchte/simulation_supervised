#!/bin/bash

######################################################
# Define settings and start training
if [ -z $1 ] ; then
  TAG="testing"
else
  TAG="$1"
fi
if [ -z $2 ] ; then
  MODELDIR="offline_esat_ou_aux_joint/2017-06-29_1313"
else
  MODELDIR="$2"
fi
if [ -z $3 ] ; then
  NUMBER_OF_RUNS=100
else
  NUMBER_OF_RUNS="$3"
fi
PARAMS="${@:4}"

# DEBUG
# TAG="test_train_and_eva"
# # MODELDIR="mobilenet"
# MODELDIR="depth"
# # PARAMS="--depth_input True --continue_training True --network fc_control --auxiliary_depth False"
# PARAMS="--continue_training False --model_path depth_net_checkpoint --network depth --auxiliary_depth False"
# NUMBER_OF_EPISODES=10
rm -r $HOME/tensorflow/log/$TAG

/bin/bash $(rospack find simulation_supervised)/scripts/train_model.sh $TAG $MODELDIR $((NUMBER_OF_RUNS/4)) 0 $PARAMS
PARAMS="$(echo $PARAMS | sed 's/--continue_training False//')"
PARAMS="$(echo $PARAMS | sed 's/--continue_training True//')"
PARAMS="$PARAMS --continue_training True"  
/bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 1 $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/train_model.sh $TAG $TAG $((NUMBER_OF_RUNS/4)) $((NUMBER_OF_RUNS/4)) $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 1 $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/train_model.sh $TAG $TAG $((NUMBER_OF_RUNS/4)) $((2*NUMBER_OF_RUNS/4)) $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 1 $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/train_model.sh $TAG $TAG $((NUMBER_OF_RUNS/4)) $((3*NUMBER_OF_RUNS/4)) $PARAMS
/bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 50 $PARAMS
# episode=1
# while [[ $episode -lt $NUMBER_OF_EPISODES ]] ;
# do
#   PARAMS="$(echo $PARAMS | sed 's/--continue_training False//')"
#   PARAMS="$(echo $PARAMS | sed 's/--continue_training True//')"
#   PARAMS="$PARAMS --continue_training True"
# 	/bin/bash $(rospack find simulation_supervised)/scripts/train_model.sh $TAG $MODELDIR 31 $PARAMS
#   /bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 10 $PARAMS
#   episode=$((episode+1))
# done
#   /bin/bash $(rospack find simulation_supervised)/scripts/evaluate_model.sh $TAG $TAG 50 $PARAMS


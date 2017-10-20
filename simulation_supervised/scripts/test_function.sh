#!/bin/bash

run_ros(){
	echo "Start run: $run_type"
	run=0
	while [ $run -lt $NUMBER_OF_FLIGHTS ] ; do
		echo "run: $run "
		run=$((run+1))
	done
}
episode=0
while [ $episode -lt 10 ] ; do
	echo 'Episode '$episode
	NUMBER_OF_FLIGHTS=5
	run_type='train'
	run_ros
	NUMBER_OF_FLIGHTS=2
	run_type='eva'
	run_ros
	episode=$((episode+1))
done
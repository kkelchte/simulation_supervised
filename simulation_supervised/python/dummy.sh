#!/bin/bash
count=0
while [ $count -lt 60 ] ; do
	sleep 1
	#echo $count
	count=$((count+1))
done


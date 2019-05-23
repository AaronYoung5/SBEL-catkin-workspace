#!/bin/bash

if test "$#" -ne 1; then
	echo "Must have exactly two arguments"
else
 	git add .
 	git commit -m "$1"
	git push https://github.com/AaronYoung5/SBEL-catkin-workspace.git master
fi

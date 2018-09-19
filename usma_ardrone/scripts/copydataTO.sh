#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
TO=$2

rsync -avI  -e ssh ~/ros/data/$DATE/ benjamin@$TO.local:/home/benjamin/ros/data/$DATE/

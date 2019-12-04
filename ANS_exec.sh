#!/bin/bash
# rsync using variables

SOURCEDIR="/home/ans-sysadmin/"
echo "${SOURCEDIR}"
ACTION_TIME=`date +"[%d-%m-%y %T]:"`
#COPY_LOG=COPY-`date +"%d-%m-%y_%T"`.log

echo "${ACTION_TIME} Starting copy task"
cd "$SOURCEDIR/OR3_wk"
catkin_make
mv "$SOURCEDIR/OR3_wk/roboteq_lib" "$SOURCEDIR/OR3_wk/src/"
catkin_make
mv "$SOURCEDIR/OR3_wk/roboteq_driver" "$SOURCEDIR/OR3_wk/src/"
mv "$SOURCEDIR/OR3_wk/roboteq_diagnostics" "$SOURCEDIR/OR3_wk/src/"
mv "$SOURCEDIR/OR3_wk/roboteq_msgs" "$SOURCEDIR/OR3_wk/src/"
catkin_make
#source `"devel/setup.sh"`

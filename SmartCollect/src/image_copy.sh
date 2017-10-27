#!/bin/sh

##name
name_string=$(find /home/dun/catkin_ws/ -cmin -50 -cmin +0.1 -name *.raw)

##md5sum
md5sum ${name_string} >> /home/dun/catkin_ws/src/name_string.md5

##move
mv ${name_string} /media/dun/15060787083_HPD/SaveRawTS/




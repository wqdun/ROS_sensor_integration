#!/bin/dash

str=''
gdalinfo_location=$(whereis test | awk -F: '{print $2}')
if [ -n "$str" ]; then
    echo "str not empty."
fi
if [ -z "$str" ]; then
    echo "str empty."
fi

if [ "AA$gdalinfo_location" = "AA" ]; then
    echo "str empty."
fi


if [ "AA$gdalinfo_location" != "AA" ]; then
    echo "str not empty."
fi
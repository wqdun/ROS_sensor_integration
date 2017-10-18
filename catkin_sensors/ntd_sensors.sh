#!/bin/sh

if [ ! -f "devel/bash.sh" ]; then
    echo "$(pwd) is not a valid project path."
    # touch "$file"
fi



rosrun imupac


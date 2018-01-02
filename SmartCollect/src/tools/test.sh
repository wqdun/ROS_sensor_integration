export log_file="/tmp/log"

. ./test_lib.sh

if [ ! -f test_lib.sh ]; then
    echo "[ERROR] ${dir_old} is not a valid project path."
    exit -1
fi

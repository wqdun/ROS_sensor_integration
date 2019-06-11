#!/bin/bash
clear
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*"
}

# https://askubuntu.com/questions/896935/how-to-make-zenity-transient-parent-warning-disappear-permanently
zenity() {
    /usr/bin/zenity "$@" 2>/dev/null
}

calc_progress() {
    local src_files_num=$1
    local dest_dir=$2
    local dest_dir_num=$(find ${dest_dir} -type f | wc -l)

    # log_with_time "$FUNCNAME start, src_files_num: ${src_files_num}; dest_dir_num: ${dest_dir_num}"
bc << EOF
    scale = 4
    ${dest_dir_num} / ${src_files_num} * 100
EOF
}

show_progress_bar() {
    local src_files_num=$1
    local dest_dir=$2

    # calc_progress ${src_files_num} ${dest_dir}
    (
        while [ 1 ]; do
            local progress=$(calc_progress ${src_files_num} ${dest_dir})
            echo ${progress} | tee -a /tmp/tmp
            # calc_progress ${src_files_num} ${dest_dir}
            echo "#Synchronizing..."
            sleep 4
        done
    ) |
    zenity --progress --title="-->${dest_dir}" --percentage=0
}

main() {
    log_with_time "$FUNCNAME start, param: $*"

    local src_dir=$(zenity --file-selection --directory --title="Select Source Folder:")
    if [ "AA${src_dir}" = "AA" ]; then
        log_with_time "Quit..."
        exit 1
    fi

    local dest_dir=$(zenity --file-selection --directory --title="Select Target Folder:")
    if [ "AA${dest_dir}" = "AA" ]; then
        log_with_time "Quit..."
        exit 1
    fi

    killall rsync
    /usr/bin/rsync -a "${src_dir}" "${dest_dir}" &

    local src_files_num=$(find ${src_dir} -type f | wc -l)
    show_progress_bar ${src_files_num} ${dest_dir}

    echo "End"
    killall rsync

    return 0
}


main $@
exit $?



# #1，自建进度条


# #进度条内上要显示的内容
# (
#     echo "15"; sleep 1
#     echo "# first step ..."; sleep 1
#     echo "35"; sleep 1
#     echo "# second step..."; sleep 1
#     echo "65"; sleep 1
# #这句不加#号，则不会在进度条上显示
#     echo " third step"; sleep 1
#     echo "85"; sleep 1
#     echo "# fourth step"; sleep 1
#     echo "100"; sleep 1
# ) |
# # percentage是进度条的起始点，auto-close是进度条达到100则自动关闭
# zenity --progress --title="test" --text="test"  --percentage=0 --auto-close
# #2，自建信息对话框，将这些内容显示到对话框中
# (
#     echo "first step show"
#     echo "second step show"
#     echo "third step no show"
#     echo "fourth step show"
# ) |
# zenity --text-info
# #用于top比较好
# top | zenity --text-info
# #3，建立一个警告对话框
# zenity --waring -text="not good"
# #4，自建滑动块,可以接受其返回值，然后做一些操作
# zenity --scale --text="test" --min-value=2 --max-value=100 --step 2
# #5，输入对话框，可以输入信息，根据返回值去做一些相应的操作
# res=$(zenity --entry --text="what is your name?");
# #6，错误对话框
# zenity --error --text="bad"

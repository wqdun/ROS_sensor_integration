#!/bin/bash
clear
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
result_log=/tmp/${script_name}".log"
detail_log=$absolute_script_path"/"$script_name".detail.log"
cp /dev/null $result_log
cp /dev/null $detail_log
passphrase="123"

check_before_install() {
    echo "$FUNCNAME start."
    if [ $user = "root" ]; then
        echo "[ERROR] You should not run ${script_name} with root."
        exit 1
    fi
    echo "I am ${user} in ${dir_old}."

    echo "Checking Internet connection..."
    ping -c1 baidu.com >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "[ERROR] No connection to Internet."
        exit 1
    fi
    echo "Check connection successfully."

    return
}

install_ros_indigo() {
    clear
    echo "$FUNCNAME start."
    local err=0
    source ${HOME}/.bashrc
    which roscore >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "ROS already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    echo "Installing ROS indigo..."
    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo apt-get install ros-indigo-desktop-full
    let err+=$?

    local source_list_file="/etc/ros/rosdep/sources.list.d/20-default.list"
    if [ -f "$source_list_file" ]; then
        echo ${passphrase} | sudo -S su >/dev/null 2>&1
        sudo mv "$source_list_file" /tmp/
    fi

    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo rosdep init && rosdep update
    let err+=$?

    grep "/opt/ros/indigo/setup.bash" ${HOME}/.bashrc >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "source /opt/ros/indigo/setup.bash" >> ${HOME}/.bashrc
        source ${HOME}/.bashrc
    fi
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

create_catkin_ws() {
    clear
    echo "$FUNCNAME start."
    local err=0
    if [ "AA${absolute_script_path}" = "AA${HOME}/catkin_ws" ]; then
        echo "You've already copy src to ${HOME}/catkin_ws, so no need copy again."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    if [ -d "${HOME}/catkin_ws" ]; then
        now=$(date +%Y_%m_%d_%H_%M_%S)
        read -n1 -p "${HOME}/catkin_ws already exist, Y for backup to catkin_ws_${now}, n for quit [Y/n]? " answer
        case $answer in
        Y | y)
            echo
            mv ${HOME}/catkin_ws ${HOME}/catkin_ws_${now};;
        N | n)
            echo
            exit 1;;
        *)
            echo
            echo "[ERROR] Wrong input, exit."
            exit 1;;
        esac
    fi

    mkdir -p ${HOME}/catkin_ws
    echo "Copy files to ${HOME}/catkin_ws, this takes time..."
    cp -r ${absolute_script_path}/src ${absolute_script_path}/data ${HOME}/catkin_ws

    write_result $FUNCNAME" returns "$err.
    return $err
}

dos_to_unix_etc() {
    clear
    echo "$FUNCNAME start."
    local err=0

    find ${HOME}/catkin_ws -name "*.sh" | xargs sed -i 's/\r//'
    let err+=$?
    find ${HOME}/catkin_ws -name "*.py" | xargs sed -i 's/\r//'
    let err+=$?

    find ${HOME}/catkin_ws -name "*.sh" | xargs chmod +x
    let err+=$?
    find ${HOME}/catkin_ws -name "*.cfg" | xargs chmod +x
    let err+=$?

    return $err
}

compile_ws() {
    clear
    echo "$FUNCNAME start."
    local err=0

    dos_to_unix_etc
    let err+=$?
    if [ $err -ne 0 ]; then
        write_result $FUNCNAME" returns "$err.
        return
    fi

    (
        cd ${HOME}/catkin_ws/src/cv_bridge/
        cmake . && make
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to make cv_bridge."
            return 1
        fi

        cd ${HOME}/catkin_ws
        catkin_make --pkg cv_bridge
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to catkin_make cv_bridge."
            return 2
        fi

        for (( i=0; i<3; i++ ))
        do
            rosdep install --from-paths src --ignore-src --rosdistro indigo -y && catkin_make
            if [ $? -eq 0 ]; then
                echo "catkin_make successfully."
                return 0
            fi
        done
        echo "[ERROR] Failed to catkin_make."
        return 3
    )
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}


install_velodyne_essential() {
    clear
    echo "$FUNCNAME start."
    local err=0
    dpkg -l | grep ros-indigo-velodyne >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "ros-indigo-velodyne already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo apt-get install ros-indigo-velodyne
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

install_opencv() {
    clear
    echo "$FUNCNAME start."
    local err=0
    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo apt-get install build-essential
    let err+=$?
    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo apt-get install cmake libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    let err+=$?

    local opencv_ver=$(opencv_version)
    if [ "AA$opencv_ver" = "AA3.3.0" ]; then
        echo "opencv-3.3.0 already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    unzip -o ${absolute_script_path}/3rd/opencv-3.3.0.zip -d /tmp/ >/dev/null 2>&1
    let err+=$?
    if [ $err -ne 0 ]; then
        write_result $FUNCNAME" returns "$err.
        return
    fi

    (
        cd /tmp/opencv-3.3.0 && mkdir -p build && cd build
        cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to cmake opencv."
            return 1
        fi
        make
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to make opencv."
            return 2
        fi
        echo ${passphrase} | sudo -S su >/dev/null 2>&1
        sudo make install
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to install opencv."
            return 3
        fi
    )

    let err+=$?
    if [ $err -ne 0 ]; then
        write_result $FUNCNAME" returns "$err.
        return
    fi

    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
    sudo ldconfig
    sudo sh -c 'echo "PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" >> /etc/bash.bashrc'
    sudo sh -c 'echo "export PKG_CONFIG_PATH" >> /etc/bash.bashrc'
    source /etc/bash.bashrc

    write_result $FUNCNAME" returns "$err.
    return
}

# remove older version cv_bridge (it is auto installed with ROS-indigo)
remove_older_cv_bridge() {
    clear
    echo "$FUNCNAME start."
    local err=0
    dpkg -l | grep ros-indigo-cv-bridge >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "ros-indigo-cv-bridge already removed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    sudo apt-get remove ros-indigo-cv-bridge
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

install_FlyCapture2() {
    clear
    echo "$FUNCNAME start."
    local err=0
    which FlyCap2 >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "FlyCapture2 already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    (
        for (( i=0; i<2; i++ ))
        do
            echo ${passphrase} | sudo -S su >/dev/null 2>&1
            sudo apt-get install libgtkglextmm-x11-1.2-dev
            if [ $? -eq 0 ]; then
                echo "Install libgtkglextmm successfully."
                return 0
            fi
            echo ${passphrase} | sudo -S su >/dev/null 2>&1
            sudo apt-get install -f
        done
        echo "[ERROR] Failed to install libgtkglextmm."
        return 1
    )
    let err+=$?

    if [ $err -ne 0 ]; then
        write_result $FUNCNAME" returns "$err.
        return
    fi

    tar zxvf ${absolute_script_path}"/3rd/flycapture2-2.9.3.43-amd64-pkg.tgz" -C /tmp/
    (
        cd /tmp/flycapture2-2.9.3.43-amd64/
        for (( i=0; i<2; i++ ))
        do
            bash install_flycapture.sh
            if [ $? -eq 0 ]; then
                echo "bash install_flycapture.sh successfully."
                return 0
            fi
            echo ${passphrase} | sudo -S su >/dev/null 2>&1
            sudo apt-get install -f
        done
        echo "[ERROR] Failed to catkin_make."
        return 1
    )
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

install_teamviewer() {
    clear
    echo "$FUNCNAME start."
    local err=0
    dpkg -l | grep teamviewer >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "teamviewer already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    (
        for (( i=0; i<2; i++ ))
        do
            echo ${passphrase} | sudo -S su >/dev/null 2>&1
            sudo dpkg -i ${absolute_script_path}/3rd/teamviewer_12.0.85001_i386.deb
            if [ $? -eq 0 ]; then
                echo "Install teamviewer_12 successfully."
                return 0
            fi
            echo ${passphrase} | sudo -S su >/dev/null 2>&1
            sudo apt-get install -f
        done
        echo "[ERROR] Failed to install teamviewer_12."
        return 1
    )
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

install_rs422_driver() {
    clear
    echo "$FUNCNAME start."
    local err=0
    write_result $FUNCNAME" returns "$err.
    return
}

install_glog() {
    clear
    echo "$FUNCNAME start."
    local err=0
    dpkg -l | grep libgoogle-glog-dev >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "glog already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    sudo apt-get install libgoogle-glog-dev
    let err+=$?
    write_result $FUNCNAME" returns "$err.
    return
}

install_sqlite3() {
    clear
    echo "$FUNCNAME start."
    local err=0
    write_result $FUNCNAME" returns "$err.
    return
}

install_gdal() {
    clear
    echo "$FUNCNAME start."
    local err=0

    local gdalinfo_location=$(whereis gdalinfo | awk -F: '{print $2}')
    if [ -n $gdalinfo_location ]; then
        echo "gdal already installed."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    tar zxvf ${absolute_script_path}"/3rd/gdal-2.2.3.tar.gz" -C /tmp/
    (
        cd /tmp/gdal-2.2.3/
        ./configure && make -j8
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to configure && make gdal."
            return 1
        fi

        echo ${passphrase} | sudo -S su >/dev/null 2>&1
        sudo make install && sudo ldconfig
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to make install && ldconfig gdal."
            return 2
        fi
        echo "Install gdal successfully."
        return 0
    )
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

deploy_ui() {
    clear
    echo "$FUNCNAME start."
    local err=0
    g++ $absolute_script_path"/src/tools/smart_collector_ui/execlt.cpp" -o SmartCollector && cp SmartCollector $HOME"/Desktop/"
    let err+=$?

    cp $absolute_script_path"/src/tools/task_config.json" $HOME"/Desktop/"
    let err+=$?

    mkdir -p $HOME"/.rviz/"
    cp $absolute_script_path"/src/tools/smart_collector_ui/default.rviz" $HOME"/.rviz/"
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}

roscore_startup() {
    clear
    echo "$FUNCNAME start."
    local err=0

    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sed -i 's/\r//' ${absolute_script_path}"/src/tools/ros_daemon.bash" \
        && sudo cp ${absolute_script_path}"/src/tools/ros_daemon.bash" /etc/init.d/ \
        && sudo chmod +x /etc/init.d/ros_daemon.bash
    let err+=$?
    if [ $err -ne 0 ]; then
        echo "[ERROR] Failed to cp && chmod ros_daemon.bash."
        write_result $FUNCNAME" returns "$err.
        return
    fi

    echo ${passphrase} | sudo -S su >/dev/null 2>&1
    sudo update-rc.d -f ros_daemon.bash remove
    sudo update-rc.d ros_daemon.bash defaults 90 10
    let err+=$?

    write_result $FUNCNAME" returns "$err.
    return
}


write_result() {
    echo $@ >>${result_log}
}

print_results() {
    clear
    echo -e "\n\t\tInstallation Results: 0 means succeeded, otherwise failed.\n"
    cat -n ${result_log}
    return
}

main() {
    check_before_install

    install_teamviewer

    install_ros_indigo
    install_velodyne_essential
    install_opencv
    remove_older_cv_bridge
    install_FlyCapture2

    install_glog
    install_rs422_driver
    install_sqlite3
    install_gdal

    roscore_startup

    create_catkin_ws
    compile_ws
    deploy_ui

    print_results
    return
}

main $@ 2>&1 | tee $detail_log
exit $?

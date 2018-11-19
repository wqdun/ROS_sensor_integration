# ubuntu14.04 + opencv 3.3 安装教程
## 0.安装配置
  本教程实用ubuntu14.04 + 64位 + opencv3.3.0
## 1.下载opencv3.3
### 1.1 下载链接
下载方法1：
http://www.opencv.org/

下载方法2 :
// git clone https://github.com/Itseez/opencv/archive/3.3.0.zip
wget https://github.com/Itseez/opencv/archive/3.3.0.zip
### 1.2 解压
找到下载的安装包opencv-3.3.0.tar.gz(或者opencv-3.3.0.zip)

解压到指定目录，本人解压目录为：/home/liuyusen/opencv-3.3.0/

## 2.opencv库安装
### 2.1 依赖项列表
#### (1)需要安装的依赖项列表如下：
GCC 4.4.x or later

CMake 2.8.7 or higher

Git

GTK+2.x or higher, including headers (libgtk2.0-dev)

pkg-config

Python 2.6 or later and Numpy 1.5 or later with developer packages (python-dev, python-numpy)

ffmpeg or libav development packages: libavcodec-dev, libavformat-dev, libswscale-dev

[optional] libtbb2 libtbb-dev

[optional] libdc1394 2.x

[optional] libjpeg-dev, libpng-dev, libtiff-dev, libjasper-dev, libdc1394-22-dev

[optional] CUDA Toolkit 6.5 or higher

### 2.2 安装依赖库
#### (1)搭建编译环境：
sudo apt-get install build-essential
#### (2)安装关联库【必须】：
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
#### (3)安装关联库【可选】：
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
### 2.3 编译源文件
#### (1)终端中切换到源文件路径：
cd /home/liuyusen/opencv-3.3.0/
#### (2)创建文件夹：
mkdir build

cd build
#### (3)文件编译：
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
#### (4)编译生成：
make
#### (5)安装opencv库到系统：
sudo make install


## 3.环境配置
### (1)配置opencv 库路径：
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'

sudo ldconfig

sudo gedit /etc/bash.bashrc

最后一行添加：

PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

export PKG_CONFIG_PATH


### (2)查看opencv 配置情况：
#### 方法1:
gedit /usr/local/lib/pkgconfig/opencv.pc
#### 方法2:
pkg-config opencv --cflags --libs

### (3)配置文件具体如下：
Package Information for pkg-config

prefix=/usr/local

exec_prefix=${prefix}

libdir=${exec_prefix}/lib

includedir_old=${prefix}/include/opencv

includedir_new=${prefix}/include


Name: OpenCV

Description: Open Source Computer Vision Library

Version: 3.3.0

Libs: -L${exec_prefix}/lib -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_viz -lopencv_core
Libs.private: -lgtk-x11-2.0 -lgdk-x11-2.0 -latk-1.0 -lgio-2.0 -lpangoft2-1.0 -lpangocairo-1.0 -lgdk_pixbuf-2.0 -lcairo -lpango-1.0 -lfontconfig -lgobject-2.0 -lglib-2.0 -lfreetype -lgthread-2.0 -L/usr/lib/x86_64-linux-gnu -lpng -lz -ltiff -ljasper -ljpeg -lImath -lIlmImf -lIex -lHalf -lIlmThread -ldc1394 -lavcodec -lavformat -lavutil -lswscale -L/usr/lib -lvtkCommon -lvtkFiltering -lvtkImaging -lvtkGraphics -lvtkGenericFiltering -lvtkIO -lvtkRendering -lvtkVolumeRendering -lvtkHybrid -lvtkWidgets -lvtkParallel -lvtkInfovis -lvtkGeovis -lvtkViews -lvtkCharts -ldl -lm -lpthread -lrt

Cflags: -I${includedir_old} -I${includedir_new}

#按照配置文件路径查看opencv相关头文件/库文件是否存在，以确保安装正确。

## 4.案例输出
### (1)编写test.cpp 如下：

#include<opencv2/core.hpp>

#include<opencv2/highgui.hpp>

#include<opencv2/imgproc.hpp>

#include <iostream>


using namespace cv;

using namespace std;

int main()
{
    Mat image;

    cout<<"imgout"<<endl;
    image = imread("./lena.jpg", 1);


    namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}

### (2)配置文件CMakeList.txt(test.cpp同一目录下)编写如下：

#### 方法1:

FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(test test.cpp)

target_link_libraries(test ${OpenCV_LIBS})

cmake_minimum_required(VERSION 2.8)

#### 方法2:
cmake_minimum_required(VERSION 2.8)


LINK_LIBRARIES("/usr/local/lib/libopencv_imgcodecs.so" "/usr/local/lib/libopencv_core.so" "/usr/local/lib/libopencv_highgui.so" "libopencv_imgproc.so")


add_executable(test test.cpp)


### (3)编译链接
#### 方法1:
cd proubuntu/opencvtest

cmake .

make

./test

#### 方法2:
g++ test.cpp -o test `pkg-config --libs --cflags opencv`

./test


参考：
g++ -L /usr/local/lib -o "Example.cpp" ./example.o -lopencv_nonfree -lopencv_objdetect -lopencv_features2d -lopencv_imgproc -lopencv_highgui -lopencv_core


#### (4)显示结果如下：
2017-09-15 12_31_17____________opencvtest.png



## 5.cv_bridge依赖opencv版本的问题
### 5.1 问题描述
    ROS（indigo版本）中cv_bridge库依赖opencv库，卸载opencv重新安装Version3.3之后，依赖cv_bridge软件包编译失败。
### 5.2解决方法
#### (1)卸载旧版本cv_bridge:
sudo apt-get remove ros-indigo-cv-bridge
#### (2)下载新版本cv_bridge:
git clone https://github.com/ros-perception/vision_opencv.git
#### (3)工作空间中重新编译cv_bridge包：
cmake .
make
catkin_make --pkg cv_bridge

## 6.编译samples
### (1)切换路径：
cd /opencv-3.3.0/samples/
cmake .
make
### (2)运行DEMO：
cd cpp
./cpp-tutorial-Sobel_Demo
### (3)运行结果：
2017-09-15 14_12_34____________demoopencv.png
2017-09-15 14_19_56____________sample.png



## 7.参考资源
【1】http://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

【2】http://www.learnopencv.com/how-to-compile-opencv-sample-code/


## 8.ROS编译cv_bridge时的问题：

报错：
/home/navi/catkin_ws/src/roscameragpsimg/src/PGCamera.h:3:25: fatal error: FlyCapture2.h: No such file or directory
需要安装：flycapture2-2.9.3.43-amd64-pkg.tgz

执行如下命令：
tar -zxvf flycapture2-2.9.3.43-amd64-pkg.tgz
./install_flycapture.sh

### 8.1 使用如下命令解决依赖问题（可能需要多次执行）：
sudo apt-get -f install

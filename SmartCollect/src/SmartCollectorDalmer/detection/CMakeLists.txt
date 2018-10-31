set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CAFFE_INCLUDE_DIR "/home/nvidia/caffe/include/")
set(CAFFE_INCLUDE_DIR_BUILD "/home/nvidia/caffe/build/include/")
set(CAFFE_LIBS_DIR "/home/nvidia/caffe/build/lib")


#set ( PROJECT_LINK_LIBS libpthread.so libcaffe.so libopencv_ml.so libopencv_cudawarping.so libopencv_cudaimgproc.so libopencv_cudaobjdetect.so libglog.so libgflags.so libcudart.so )
set ( PROJECT_LINK_LIBS libpthread.so libcaffe.so libopencv_ml.so  libglog.so libgflags.so libcudart.so )

set ( CMAKE_CXX_FLAGS_DEBUG "-g -O0" )
set ( CMAKE_CXX_FLAGS_RELEASE "-g" )
set ( CMAKE_CXX_STANDARD 11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )

aux_source_directory(. /home/sen/projects/SmartCollectorDalmer/detection)

include_directories(${CAFFE_INCLUDE_DIR} ${CAFFE_INCLUDE_DIR_BUILD} /usr/local/cuda-8.0/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})

link_directories(${CAFFE_LIBS_DIR} /usr/local/cuda-8.0/lib64)

set(SOURCE_FILES  ./detection.h ./detection.cpp )

add_definitions(-Wall)
add_definitions(-DUSE_OPENCV)
add_definitions(-DUSE_CUDA)

#add_executable(DETECTION ${SOURCE_FILES})

add_library(detection ${SOURCE_FILES})
target_link_libraries(detection ${PROJECT_LINK_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

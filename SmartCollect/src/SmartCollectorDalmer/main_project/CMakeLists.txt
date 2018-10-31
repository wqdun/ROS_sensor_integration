set(CMAKE_CXX_STANDARD 11)
set(CAFFE_INCLUDE_DIR "/home/nvidia/caffe/include/")
set(CAFFE_INCLUDE_DIR_BUILD "/home/nvidia/caffe/build/include/")
set(CAFFE_LIBS_DIR "/home/nvidia/caffe/build/lib")


#set ( PROJECT_LINK_LIBS libpthread.so libcaffe.so libopencv_ml.so libopencv_cudawarping.so libopencv_cudaimgproc.so libopencv_cudaobjdetect.so libglog.so libgflags.so libcudart.so )
set ( PROJECT_LINK_LIBS libpthread.so libcaffe.so libopencv_ml.so  libglog.so libgflags.so libcudart.so )
set ( CMAKE_CXX_FLAGS_DEBUG "-g -O0" )
set ( CMAKE_CXX_FLAGS_RELEASE "-g" )
set ( CMAKE_CXX_STANDARD 11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )

aux_source_directory(. /home/nvidia/projects/SmartCollectorDalmer/main_project)

include_directories(${CAFFE_INCLUDE_DIR} ${CAFFE_INCLUDE_DIR_BUILD} /usr/local/cuda-8.0/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})

link_directories(${CAFFE_LIBS_DIR} /usr/local/cuda-8.0/lib64)


set(SOURCE_FILES  ./data_input.h ./data_input.cpp ./data_output.h ./data_output.cpp)

add_definitions(-Wall)
add_definitions(-DUSE_OPENCV)
add_definitions(-DUSE_CUDA)

#add_executable(MAINIO ${SOURCE_FILES})
#target_link_libraries(MAINIO ${PROJECT_LINK_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(main_project ${SOURCE_FILES})

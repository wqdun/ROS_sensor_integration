cmake_minimum_required(VERSION 2.8.3)
project(test)
message(STATUS "PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

# SET(CMAKE_C_COMPILER "arm-hisiv500-linux-gcc")
# SET(CMAKE_CXX_COMPILER "arm-hisiv500-linux-g++")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add_compile_options(-std=c++11)
# add_compile_options(-Wall)
# add_compile_options(-Dhi3521a -DHICHIP=0x3521a100 -DHI_RELEASE -DHI_XXXX -DHI_ACODEC_TYPE_NVP6124 -DHI_ACODEC_TYPE_HDMI)

# include_directories(
#   ${PROJECT_SOURCE_DIR}
#   ${PROJECT_SOURCE_DIR}/include
#   ${PROJECT_SOURCE_DIR}/extdrv/tlv320aic31
#   ${PROJECT_SOURCE_DIR}/extdrv/nvp6124
# )
add_executable(${PROJECT_NAME}_node
  ${PROJECT_SOURCE_DIR}/test.cc
)
add_definitions(-DNOTEST)
add_executable(${PROJECT_NAME}_node_notest
  ${PROJECT_SOURCE_DIR}/test.cc
)

# target_link_libraries(${PROJECT_NAME}_node
#   pthread
#   m
#   dl
#   opencv_core
#   opencv_highgui
#   opencv_imgcodecs
#   opencv_imgproc
#   opencv_objdetect
#   ${PROJECT_SOURCE_DIR}/lib/libmpi.a
#   ${PROJECT_SOURCE_DIR}/lib/libhdmi.a
#   ${PROJECT_SOURCE_DIR}/lib/libVoiceEngine.a
#   ${PROJECT_SOURCE_DIR}/lib/libupvqe.a
#   ${PROJECT_SOURCE_DIR}/lib/libdnvqe.a
#   ${PROJECT_SOURCE_DIR}/lib/libjpeg.a
#   ${PROJECT_SOURCE_DIR}/lib/libdlib.a
# )


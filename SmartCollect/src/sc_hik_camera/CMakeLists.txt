cmake_minimum_required(VERSION 3.5)
project(VINS_pc)

#define build type
IF(NOT CMAKE_BUILD_TYPE)
#    SET(CMAKE_BUILD_TYPE ReleaseWD)
    SET(CMAKE_BUILD_TYPE ReleaseWD)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -march=native")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")  # -pthread is for using thread
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#List cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#for opencv 3
# http://blog.csdn.net/thornsuge/article/details/71171272
# set(OpenCV_DIR "/home/nvidia/opencv320install/share/OpenCV")

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Ceres REQUIRED)

find_package(Eigen3  REQUIRED)

find_package(Pangolin REQUIRED)

# find_package(SuiteSparse REQUIRED)

#set(${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)

MESSAGE("Build type: " ${OPENCV_INCLUDE_DIRS})


MESSAGE("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

#find_package(Glog REQUIRED)
#include_directories(BEFORE ${GLOG_INCLUDE_DIRS})
include_directories(
        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/src/camera2/include
        ${PROJECT_SOURCE_DIR}/src/estimator
        ${PROJECT_SOURCE_DIR}/src/factor
        ${PROJECT_SOURCE_DIR}/src/system
        ${PROJECT_SOURCE_DIR}/src/feature
        ${PROJECT_SOURCE_DIR}/src/initial
        ${PROJECT_SOURCE_DIR}/src/loop
        ${PROJECT_SOURCE_DIR}/src/utility
        ${PROJECT_SOURCE_DIR}/src/visualization
        ${PROJECT_SOURCE_DIR}/src/loop/ThirdParty/DBoW
        ${PROJECT_SOURCE_DIR}/src/loop/ThirdParty/DUtils
        ${PROJECT_SOURCE_DIR}/src/loop/ThirdParty/DVision
        ${PROJECT_SOURCE_DIR}/src/loop/ThirdParty
        #${PROJECT_SOURCE_DIR}/src/system
        ${PROJECT_SOURCE_DIR}/src/viewer
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${SUITESPARSE_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        /opt/mvs/include
        /home/nvidia/projects/SmartCollectorDalmer/detection
        /home/nvidia/projects/SmartCollectorDalmer/detection_cpu
        /home/nvidia/projects/SmartCollectorDalmer/main_project
        /home/nvidia/projects/SmartCollectorDalmer/classification
	/home/nvidia/caffe/include/
        /usr/local/cuda-8.0/targets/aarch64-linux/include/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

######################## DUtils part######################################
add_library(DUtils SHARED
        src/loop/ThirdParty/DUtils/Random.cpp
        src/loop/ThirdParty/DUtils/Timestamp.cpp
        )

target_link_libraries(DUtils ${Boost_LIBRARIES})
##########################################################################

####################### DVision part ####################################
add_library(
    DVision SHARED src/loop/ThirdParty/DVision/BRIEF.cpp
)
target_link_libraries(DVision ${Boost_LIBRARIES} ${OpenCV_LIBS} DUtils)
##########################################################################

###################### DBOW part #########################################
add_library(
        DBoW SHARED src/loop/ThirdParty/DBoW/BowVector.cpp src/loop/ThirdParty/DBoW/FBrief.cpp
        src/loop/ThirdParty/DBoW/FeatureVector.cpp src/loop/ThirdParty/DBoW/QueryResults.cpp
        src/loop/ThirdParty/DBoW/ScoringObject.cpp src/loop/ThirdParty/VocabularyBinary.cpp
)
target_link_libraries(DBoW ${Boost_LIBRARIES} ${OpenCV_LIBS} DUtils DVision)
##########################################################################

###################### main part #########################################
add_library(${PROJECT_NAME} SHARED
        src/estimator/VINS.cpp
        src/estimator/VINS.hpp

        src/factor/loop_closure_factor.cpp
        src/factor/marginalization_factor.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/projection_factor.cpp
        src/factor/projection_td_factor.cpp
        #src/factor/encoder_displacement_factor.cpp

        src/feature/feature_manager.cpp
        src/feature/feature_manager.hpp
        src/feature/feature_tracker.cpp

        src/initial/initial_sfm.cpp
        src/initial/initial_aligment.cpp
        src/initial/motion_estimator.cpp
        src/initial/initial_ex_rotation.cpp

        #src/loop/ThirdParty/DBoW/BowVector.cpp
        #src/loop/ThirdParty/DBoW/FBrief.cpp
        #src/loop/ThirdParty/DBoW/FeatureVector.cpp
        #src/loop/ThirdParty/DBoW/QueryResults.cpp
        #src/loop/ThirdParty/DBoW/ScoringObject.cpp
        #src/loop/ThirdParty/DUtils/Random.cpp
        #src/loop/ThirdParty/DUtils/Timestamp.cpp
        #src/loop/ThirdParty/DVision/BRIEF.cpp
        #src/loop/ThirdParty/VocabularyBinary.cpp
        src/loop/keyframe_database.cpp
        src/loop/keyframe.cpp
        src/loop/loop_closure.cpp

        src/utility/utility.cpp
        #src/visualization/viewer.cpp src/visualization/viewer.h
        #src/initial/initial_ex_rotation.cpp src/initial/initial_ex_rotation.h

        src/system/system.h
        src/system/system.cpp  src/viewer/Viewer.cpp src/viewer/MapDrawer.cpp src/viewer/MapDrawer.h src/integration_base2.h src/factor/YawLocalParameterization.cpp src/factor/YawLocalParameterization.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        /home/nvidia/projects/SmartCollectorDalmer/build/detection/libdetection.a
        /home/nvidia/projects/SmartCollectorDalmer/build/classification/libclassification.a
        /home/nvidia/projects/SmartCollectorDalmer/build/main_project/libmain_project.a
        /home/nvidia/projects/SmartCollectorDalmer/build/detection_cpu/libdetection_cpu.a
        /home/nvidia/caffe/build/lib/libcaffe.so
        DUtils
        DVision
        DBoW
)

add_library(${PROJECT_NAME}_nodetect SHARED
        src/estimator/VINS.cpp
        src/estimator/VINS.hpp

        src/factor/loop_closure_factor.cpp
        src/factor/marginalization_factor.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/projection_factor.cpp
        src/factor/projection_td_factor.cpp
        #src/factor/encoder_displacement_factor.cpp

        src/feature/feature_manager.cpp
        src/feature/feature_manager.hpp
        src/feature/feature_tracker.cpp

        src/initial/initial_sfm.cpp
        src/initial/initial_aligment.cpp
        src/initial/motion_estimator.cpp
        src/initial/initial_ex_rotation.cpp

        #src/loop/ThirdParty/DBoW/BowVector.cpp
        #src/loop/ThirdParty/DBoW/FBrief.cpp
        #src/loop/ThirdParty/DBoW/FeatureVector.cpp
        #src/loop/ThirdParty/DBoW/QueryResults.cpp
        #src/loop/ThirdParty/DBoW/ScoringObject.cpp
        #src/loop/ThirdParty/DUtils/Random.cpp
        #src/loop/ThirdParty/DUtils/Timestamp.cpp
        #src/loop/ThirdParty/DVision/BRIEF.cpp
        #src/loop/ThirdParty/VocabularyBinary.cpp
        src/loop/keyframe_database.cpp
        src/loop/keyframe.cpp
        src/loop/loop_closure.cpp

        src/utility/utility.cpp
        #src/visualization/viewer.cpp src/visualization/viewer.h
        #src/initial/initial_ex_rotation.cpp src/initial/initial_ex_rotation.h

        src/system/system.h
        src/system/system_nodetect.cpp  src/viewer/Viewer.cpp src/viewer/MapDrawer.cpp src/viewer/MapDrawer.h src/integration_base2.h src/factor/YawLocalParameterization.cpp src/factor/YawLocalParameterization.h)

target_link_libraries(${PROJECT_NAME}_nodetect
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        /home/nvidia/projects/SmartCollectorDalmer/build/detection/libdetection.a
        /home/nvidia/projects/SmartCollectorDalmer/build/classification/libclassification.a
        /home/nvidia/projects/SmartCollectorDalmer/build/main_project/libmain_project.a
        /home/nvidia/projects/SmartCollectorDalmer/build/detection_cpu/libdetection_cpu.a
        /home/nvidia/caffe/build/lib/libcaffe.so
        DUtils
        DVision
        DBoW
)

###################################################################################

########################## camera part ############################################
add_library(camera_model
    src/camera2/src/calib/CameraCalibration.cc
    src/camera2/src/camera_models/Camera.cc
    src/camera2/src/camera_models/CameraFactory.cc
    src/camera2/src/camera_models/CostFunctionFactory.cc
    src/camera2/src/camera_models/PinholeCamera.cc
    src/camera2/src/camera_models/CataCamera.cc
    src/camera2/src/camera_models/EquidistantCamera.cc
    src/camera2/src/camera_models/ScaramuzzaCamera.cc
    src/camera2/src/sparse_graph/Transform.cc
    src/camera2/src/gpl/gpl.cc
    src/camera2/src/gpl/EigenQuaternionParameterization.cc
)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
###########################################################################################

add_executable(system_test
    src/hik_camera_arm_node.cpp
	src/hik_camera_arm.cpp
	src/serial_reader.cpp
	src/can_parser.cpp
    ../sc_lib_public_tools/src/tools_no_ros.cpp
)

target_link_libraries(system_test
    ${PROJECT_NAME}
    camera_model
    boost_thread
    controlcan
    /opt/mvs/lib/arm64/libMvCameraControl.so
    ${PROJECT_SOURCE_DIR}/../../3rd/libcontrolcan.so
)

add_executable(system_test_nodetect
    src/hik_camera_arm_node.cpp
    src/hik_camera_arm.cpp
    src/serial_reader.cpp
    src/can_parser.cpp
    ../sc_lib_public_tools/src/tools_no_ros.cpp
)

target_link_libraries(system_test_nodetect
    ${PROJECT_NAME}_nodetect
    camera_model
    boost_thread
    controlcan
    /opt/mvs/lib/arm64/libMvCameraControl.so
    ${PROJECT_SOURCE_DIR}/../../3rd/libcontrolcan.so
)

######################## offline test ######################################
# add_executable(offline_test src/example/main_honor8.cpp)
# target_link_libraries(offline_test ${PROJECT_NAME} camera_model boost_thread)

add_executable(tx2_test src/example/main_tx2.cpp)
target_link_libraries(tx2_test ${PROJECT_NAME} camera_model boost_thread)
#add_executable(system_test_euroc   src/Example/system_test_euroc.cpp)

#target_link_libraries(system_test_euroc ${PROJECT_NAME} camera_model)

add_executable(can_killer
    src/can_killer.cc
)

target_link_libraries(can_killer
    ${PROJECT_SOURCE_DIR}/../../3rd/libcontrolcan.so
    glog
)



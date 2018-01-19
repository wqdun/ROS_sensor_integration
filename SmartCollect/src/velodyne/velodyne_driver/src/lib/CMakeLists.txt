add_compile_options(-std=c++11)
add_library(velodyne_input input.cc ../../../../public_tools/public_tools.cpp)
target_link_libraries(velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
  glog
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(velodyne_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS velodyne_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

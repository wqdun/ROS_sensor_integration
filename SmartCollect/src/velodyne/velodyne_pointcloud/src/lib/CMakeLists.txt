add_compile_options(-std=c++11)
add_library(velodyne_rawdata rawdata.cc calibration.cc)
target_link_libraries(velodyne_rawdata
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      sc_lib_public_tools)
install(TARGETS velodyne_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

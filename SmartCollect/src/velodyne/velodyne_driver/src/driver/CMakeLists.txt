add_compile_options(-std=c++11)
# build the driver node
# add_executable(velodyne_node velodyne_node.cc driver.cc)
# add_dependencies(velodyne_node velodyne_driver_gencfg)
# target_link_libraries(velodyne_node
#   velodyne_input
#   ${catkin_LIBRARIES}
#   ${libpcap_LIBRARIES}
# )

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc center_subscriber.cpp)
add_dependencies(driver_nodelet velodyne_driver_gencfg)
target_link_libraries(driver_nodelet
  ${catkin_LIBRARIES}
  velodyne_input
  ${libpcap_LIBRARIES}
  sc_lib_public_tools
)

# install runtime files
# install(TARGETS velodyne_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         COMPONENT main
# )
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

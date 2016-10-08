##############################################################################
# NODELET
##############################################################################

add_library(xbot_nodelet xbot_nodelet.cpp)
add_dependencies(xbot_nodelet xbot_ros)
target_link_libraries(xbot_nodelet xbot_ros)

install(TARGETS xbot_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
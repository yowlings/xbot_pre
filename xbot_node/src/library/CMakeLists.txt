##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(xbot_ros ${SOURCES})
add_dependencies(xbot_ros xbot_msgs_gencpp)
target_link_libraries(xbot_ros ${catkin_LIBRARIES})

install(TARGETS xbot_ros
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

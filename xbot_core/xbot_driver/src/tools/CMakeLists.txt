###############################################################################
# Tools
###############################################################################

add_executable(xbot_version_info version_info.cpp)
target_link_libraries(xbot_version_info xbot)

add_executable(xbot_simple_keyop simple_keyop.cpp)
target_link_libraries(xbot_simple_keyop xbot)

install(TARGETS xbot_version_info xbot_simple_keyop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

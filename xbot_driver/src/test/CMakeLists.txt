###############################################################################
# Tests
###############################################################################

add_executable(xbot_velocity_commands velocity_commands.cpp)
target_link_libraries(xbot_velocity_commands xbot)

add_executable(demo_xbot_initialisation initialisation.cpp)
target_link_libraries(demo_xbot_initialisation xbot)

add_executable(demo_xbot_sigslots sigslots.cpp)
target_link_libraries(demo_xbot_sigslots xbot)

add_executable(demo_xbot_simple_loop simple_loop.cpp)
target_link_libraries(demo_xbot_simple_loop xbot)

install(TARGETS xbot_velocity_commands demo_xbot_initialisation demo_xbot_sigslots demo_xbot_simple_loop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
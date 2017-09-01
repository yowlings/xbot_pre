##############################################################################
# NODELET
##############################################################################



add_executable(debugTopics debugTopics.cpp)
target_link_libraries(debugTopics ${catkin_LIBRARIES})
#add_dependencies(debugTopoics xbot_node_generate_messages_cpp)


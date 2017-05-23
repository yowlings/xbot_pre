cmake_minimum_required(VERSION 2.8.3)
project(xbot_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES 
                      SensorState.msg
                      DebugSensor.msg
                      BumperEvent.msg
                      CliffEvent.msg
                      RobotStateEvent.msg
                      PowerSystemEvent.msg
                      WheelDropEvent.msg
                      ControllerInfo.msg
                      DockInfraRed.msg
                      KeyboardInput.msg
                      MotorPower.msg
                      ScanAngle.msg
                      XbotState.msg
                      face_pose.msg
                      ImuNine.msg
                      NaviState.msg
                 )

add_action_files(DIRECTORY action
                     FILES AutoDocking.action
                )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)

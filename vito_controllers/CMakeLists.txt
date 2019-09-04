cmake_minimum_required(VERSION 2.8.3)
project(vito_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  geometry_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  kdl_conversions
  sensor_msgs
  std_msgs
  lwr_hw
  tf_conversions
  lwr_controllers
)

add_definitions (-fpermissive -std=c++11)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
  #  PoseRPY.msg
  #  RPY.msg
  #  MultiPriorityTask.msg
  #  ArmState.msg
  #  CartesianImpedancePoint.msg
  #  Stiffness.msg
)
add_service_files(
  FILES
  # SetCartesianImpedanceCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    geometry_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
    lwr_hw
    lwr_controllers
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/cartesian_impedance_vito_controller.cpp 
  src/KinematicChainControllerBase.cpp
  src/vito_bridge_controller.cpp 
  src/teleoperation_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES vito_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(dual_arm_manager src/dual_arm_manager.cpp src/DualArms.cpp)
add_dependencies(dual_arm_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_arm_manager ${catkin_LIBRARIES})

# add_executable(cartesian_command_vito src/cartesian_command_vito.cpp)

# target_link_libraries(cartesian_command_vito ${catkin_LIBRARIES})
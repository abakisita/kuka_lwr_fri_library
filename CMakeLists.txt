cmake_minimum_required(VERSION 3.5)
project(kuka_lwr_fri_library)

## Compile as C++11, supported in ROS Kinetic and newer
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

#find_package(cmake_modules) 
find_package(sensor_msgs)
#find_package(tf) 


find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


## DEPENDS: system dependencies of this project that dependent projects also need
#ament_package(
#  INCLUDE_DIRS include
#  LIBRARIES FastResearchInterfaceLibrary
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#)

## Your package locations should be listed before other locations
include_directories(
  include
  PRIVATE ${PYTHON_INCLUDE_DIRS}

)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/FastResearchInterfaceLibrary/Console.cpp                
  src/FastResearchInterfaceLibrary/GetCompleteRobotStateAndInformation.cpp  
  src/FastResearchInterfaceLibrary/GetUDPCommunicationData.cpp     
  src/FastResearchInterfaceLibrary/LinuxAbstraction.cpp  
  src/FastResearchInterfaceLibrary/SetControlScheme.cpp     
  src/FastResearchInterfaceLibrary/StartRobot.cpp       
  src/FastResearchInterfaceLibrary/UDPSocket.cpp
  src/FastResearchInterfaceLibrary/DataLogging.cpp            
  src/FastResearchInterfaceLibrary/GetRobotControlData.cpp                  
  src/FastResearchInterfaceLibrary/InitializationFileEntry.cpp     
  src/FastResearchInterfaceLibrary/LoggingMethods.cpp    
  src/FastResearchInterfaceLibrary/SetRobotControlData.cpp  
  src/FastResearchInterfaceLibrary/StopRobot.cpp        
  src/FastResearchInterfaceLibrary/WaitForTicks.cpp
  src/FastResearchInterfaceLibrary/FastResearchInterface.cpp  
  src/FastResearchInterfaceLibrary/GetRobotStatusData.cpp                   
  src/FastResearchInterfaceLibrary/KRCCommunicationThreadMain.cpp  
  src/FastResearchInterfaceLibrary/ReadInitFile.cpp      
  src/FastResearchInterfaceLibrary/SetRobotStatusData.cpp   
  src/FastResearchInterfaceLibrary/TimerThreadMain.cpp
  src/FastResearchInterfaceLibrary/MoveToCandle.cpp
  src/FastResearchInterfaceLibrary/RunTrajectorySimple.cpp
)
add_library(TypeIRML
  src/FastResearchInterfaceLibrary/TypeIRML/TypeIRML.cpp          
  src/FastResearchInterfaceLibrary/TypeIRML/TypeIRMLMath.cpp        
  src/FastResearchInterfaceLibrary/TypeIRML/TypeIRMLProfiles.cpp
  src/FastResearchInterfaceLibrary/TypeIRML/TypeIRMLDecision.cpp  
  src/FastResearchInterfaceLibrary/TypeIRML/TypeIRMLPolynomial.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

#\add_executable(cart_test_node scripts/FastResearchInterfaceTest/cart_test.cpp)
#add_executable(fri_test_node scripts/FastResearchInterfaceTest/FastResearchInterfaceTest.cpp)
#add_executable(calibration_node scripts/FastResearchInterfaceTest/calibration.cpp scripts/FastResearchInterfaceTest/ft_calib.cpp)
#add_executable(ros_test_node scripts/FastResearchInterfaceTest/ros_test.cpp)
add_executable(cart_test_2_node scripts/LWRGettingStartedExamples/LWRCartImpedanceControlExample.cpp)
#add_executable(cpp_test_node scripts/LWRGettingStartedExamples/cpp_test.cpp)
#add_executable(joint_test_node scripts/LWRGettingStartedExamples/LWRJointPositionControlExample.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}

   pthread
 )
 target_link_libraries(${PROJECT_NAME}
   pthread
 )
# target_link_libraries(cart_test_node
#   FastResearchInterfaceLibrary
#   TypeIRML
#
# )

# target_link_libraries(joint_test_node
#   FastResearchInterfaceLibrary
#   TypeIRML
#
# )


# target_link_libraries(fri_test_node
#   FastResearchInterfaceLibrary
#   TypeIRML
#   ${catkin_LIBRARIES}
# )


# target_link_libraries(calibration_node
#   FastResearchInterfaceLibrary
#   TypeIRML
#   pthread
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(ros_test_node
 #  ${catkin_LIBRARIES}
 #)

 target_link_libraries(cart_test_2_node
	 ${PROJECT_NAME}
   TypeIRML
   
 #  rclcpp
 #  std_msgs
 #  sensor_msgs
 )
ament_target_dependencies(cart_test_2_node 
      rclcpp 
      std_msgs 
      sensor_msgs
      orocos_kdl
      )
# target_link_libraries(cpp_test_node
#   FastResearchInterfaceLibrary
#   TypeIRML
#   ${catkin_LIBRARIES}
#   ${PYTHON_LIBRARIES}
# )
#target_include_directories(myproject PRIVATE ${PYTHON_INCLUDE_DIRS})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS cart_test_2_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	DESTINATION lib/${PROJECT_NAME}
	RUNTIME DESTINATION lib/${PROJECT_NAME}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fri_catkin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
ament_package()

cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_csm)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure laser_odometry_core roscpp rosparam_handler
)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

generate_ros_parameter_files(
  cfg/LaserOdometryCsm.params
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure laser_odometry_core roscpp rosparam_handler #csm
  #DEPENDS csm
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
link_directories(SYSTEM ${csm_LIBRARY_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME} src/laser_odometry_csm.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${csm_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${csm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_genparam) # For rosparam_handler

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_odometry_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch #cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################
##    Test    ##
################

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    find_package(catkin REQUIRED COMPONENTS
#        roslib
#    )

#    add_rostest_gtest(test_carmen_reader_test
#      test/stuff.test
#      test/stuff.cpp
#    )

#  target_link_libraries(test_csm_test ${catkin_LIBRARIES} ${PROJECT_NAME})

#endif()

cmake_minimum_required(VERSION 2.8.3)
project(laser_odometry_csm)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
            Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp laser_odometry_core rosparam_handler dynamic_reconfigure
)

generate_ros_parameter_files(
  cfg/LaserOdometryCsm.params
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS laser_odometry_core rosparam_handler dynamic_reconfigure #csm
)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME} src/laser_odometry_csm.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${csm_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${csm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_genparam) # For rosparam_handler

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

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

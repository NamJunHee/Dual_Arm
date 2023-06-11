##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(cv_skel_cam)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
 qt_build
 roscpp
 geometry_msgs
 std_msgs
 orocos_kdl
 roslib
 tf
 cv_bridge
 image_transport
 skeleton_message

)

find_package( OpenCV REQUIRED )

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
find_path(OpenNI2_INCLUDEDIR_M
          NAMES OpenNI.h
          HINTS /usr/include/openni2 /home/robit/OpenNI2/Include)
find_library(OpenNI2_LIBRARIES_N
             NAMES OpenNI2 DummyDevice OniFile PS1080 realsense2 rs2driver
#             HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
             HINTS /home/robit/NiTE-Linux-x64-2.2/Samples/Bin /home/robit/NiTE-Linux-x64-2.2/Samples/Bin/OpenNI2/Drivers
             PATH_SUFFIXES lib
)
message(STATUS ${OpenNI2_LIBRARIES_N})

# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR_M
          NAMES NiTE.h
          HINTS $ENV{NITE2_INCLUDE})
find_library(Nite2_LIBRARY_M
             NAMES NiTE2
             HINTS $ENV{NITE2_REDIST64}
             PATH_SUFFIXES lib)

catkin_package()



include_directories(${catkin_INCLUDE_DIRS}
	 	    ${OpenCV_INCLUDE_DIRS}
                    ${OpenNI2_INCLUDEDIR_M}
                    ${Nite2_INCLUDEDIR_M})


##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/cv_skel_cam/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(cv_skel_cam ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(cv_skel_cam ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES_N} ${Nite2_LIBRARY_M} ${OpenCV_LIBRARIES})
install(TARGETS cv_skel_cam RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


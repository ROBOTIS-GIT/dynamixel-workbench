################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.0)
project(dynamixel_workbench_single_manager_gui)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    dynamixel_workbench_msgs
    dynamixel_workbench_toolbox
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################

################################################################################
# Build
################################################################################
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

################################################################################
# Qt Environment
################################################################################
rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/dynamixel_workbench_single_manager_gui/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

add_executable(dynamixel_workbench_single_manager_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(dynamixel_workbench_single_manager_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS dynamixel_workbench_single_manager_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Build
################################################################################
install(TARGETS dynamixel_workbench_single_manager_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.0)
project(dynamixel_workbench_single_manager_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/message_header.h
  include/${PROJECT_NAME}/qnode.hpp
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_RESOURCES
  resources/images.qrc
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp dynamixel_workbench_msgs dynamixel_workbench_toolbox
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
  qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
  qt4_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${dynamixel_workbench_single_manager_gui_INCLUDE_DIRECTORIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
else()
  qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

################################################################################
# Build
################################################################################
include_directories(
  include
  ${dynamixel_workbench_single_manager_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_workbench_single_manager_gui ${dynamixel_workbench_single_manager_gui_SRCS} ${dynamixel_workbench_single_manager_gui_MOCS} ${dynamixel_workbench_single_manager_gui_UIS_H} ${dynamixel_workbench_single_manager_gui_RCC})
add_dependencies(dynamixel_workbench_single_manager_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_single_manager_gui ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#################################################################################
## Install
#################################################################################
install(TARGETS dynamixel_workbench_single_manager_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resources ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################

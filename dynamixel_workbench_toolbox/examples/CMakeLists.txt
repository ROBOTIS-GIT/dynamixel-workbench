cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench)

add_compile_options(-std=c++11)

include_directories(
  ../src
)

add_library(dynamixel_workbench
  ../src/dynamixel_workbench_toolbox/dynamixel_item.cpp
  ../src/dynamixel_workbench_toolbox/dynamixel_driver.cpp
  ../src/dynamixel_workbench_toolbox/dynamixel_tool.cpp
  ../src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
)

if(APPLE)
  target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_mac_cpp.dylib")
else()
  target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")
endif()

add_executable(model_scan src/a_Model_Scan.cpp)
target_link_libraries(model_scan LINK_PUBLIC dynamixel_workbench)

add_executable(ping src/b_Ping.cpp)
target_link_libraries(ping LINK_PUBLIC dynamixel_workbench)

add_executable(id_change src/c_ID_Change.cpp)
target_link_libraries(id_change LINK_PUBLIC dynamixel_workbench)

add_executable(bps_change src/d_BPS_Change.cpp)
target_link_libraries(bps_change LINK_PUBLIC dynamixel_workbench)

add_executable(mode_change src/e_Mode_Change.cpp)
target_link_libraries(mode_change LINK_PUBLIC dynamixel_workbench)

add_executable(monitor src/s_Monitor.cpp)
target_link_libraries(monitor LINK_PUBLIC dynamixel_workbench)

add_executable(find_dynamixel src/t_Find_Dynamixel.cpp)
target_link_libraries(find_dynamixel LINK_PUBLIC dynamixel_workbench)
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

add_executable(reboot src/f_Reboot.cpp)
target_link_libraries(reboot LINK_PUBLIC dynamixel_workbench)

add_executable(reset src/g_Reset.cpp)
target_link_libraries(reset LINK_PUBLIC dynamixel_workbench)

add_executable(position src/h_Position.cpp)
target_link_libraries(position LINK_PUBLIC dynamixel_workbench)

add_executable(velocity src/i_Velocity.cpp)
target_link_libraries(velocity LINK_PUBLIC dynamixel_workbench)

add_executable(current_based_position src/j_Current_Based_Position.cpp)
target_link_libraries(current_based_position LINK_PUBLIC dynamixel_workbench)

add_executable(read_write src/k_Read_Write.cpp)
target_link_libraries(read_write LINK_PUBLIC dynamixel_workbench)

add_executable(sync_write src/l_Sync_Write.cpp)
target_link_libraries(sync_write LINK_PUBLIC dynamixel_workbench)

add_executable(sync_read_write src/m_Sync_Read_Write.cpp)
target_link_libraries(sync_read_write LINK_PUBLIC dynamixel_workbench)

add_executable(bulk_read_write src/n_Bulk_Read_Write.cpp)
target_link_libraries(bulk_read_write LINK_PUBLIC dynamixel_workbench)

add_executable(find_dynamixel src/o_Find_Dynamixel.cpp)
target_link_libraries(find_dynamixel LINK_PUBLIC dynamixel_workbench)

add_executable(monitor src/p_Monitor.cpp)
target_link_libraries(monitor LINK_PUBLIC dynamixel_workbench)
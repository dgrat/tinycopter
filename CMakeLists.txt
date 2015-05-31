cmake_minimum_required(VERSION 2.8)
project(RPiCopter)

# Standard c++ source files
SET( tinycopter
  absdevice
  containers
  device
  exceptions
  navigation
  frame
  vehicle
  receiver
  scheduler
)
# Arduino project file (*.pde), which must be converted for gcc
SET ( PROJECT_FILE
  tinycopter
)

message( STATUS "Include ArduPilot library directories" )
foreach( DIR ${AP_List} ${AP_List_Linux} ${AP_Headers} )
	include_directories( "../libraries/${DIR}" )
endforeach()
include_directories( zserge-jsmn )

# ***************************************
# Build the firmware
# ***************************************
add_subdirectory ( zserge-jsmn )

# Exchange the file ending of the Arduino project file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tinycopter.pde ${CMAKE_CURRENT_BINARY_DIR}/tinycopter.cpp)

# Compile the project
ADD_EXECUTABLE ( TinyCopter ${tinycopter} ${PROJECT_FILE} )
target_link_libraries ( TinyCopter -Wl,--start-group ${AP_List} ${AP_List_Linux} jsmn -Wl,--end-group )
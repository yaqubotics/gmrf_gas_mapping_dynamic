cmake_minimum_required(VERSION 2.8.3)
project(gmrf_gas_mapping_dynamic)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  cv_bridge
  pcl_ros
  tf
  std_msgs
  nav_msgs  
  sensor_msgs 
  visualization_msgs
  olfaction_msgs  
  geometry_msgs
  gas_map_msgs
  message_generation
  )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS opencv2 libpcl-all-dev
    CATKIN_DEPENDS message_runtime std_msgs nav_msgs cv_bridge roscpp pcl_ros
    LIBRARIES ${PROJECT_NAME}
)


FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gmrf_gas_mapping_dynamic_node src/gmrf_node.cpp src/gmrf_map.cpp)
add_executable(gmrf_subscriber_example_node src/gmrf_subscriber_example.cpp)
add_dependencies(gmrf_gas_mapping_dynamic_node ${olfaction_msgs_EXPORTED_TARGETS} ${gas_map_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
link_directories(${catkin_LIBRARY_DIRS})

target_link_libraries(gmrf_gas_mapping_dynamic_node
   ${catkin_LIBRARIES}
 )
target_link_libraries(
   gmrf_subscriber_example_node
   ${catkin_LIBRARIES}
 )
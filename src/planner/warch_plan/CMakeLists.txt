cmake_minimum_required(VERSION 3.0.2)
project(warch_plan)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)

# set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall") #c++编译器设置 -Wextra -Werror


# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
  bspline_opt
  traj_utils
  visualization_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
   FILES
   FlyInfo.msg
   EnvInfo.msg
   GloPath.msg
   OccupyData.msg
   PathData.msg 
)
generate_messages(DEPENDENCIES
    std_msgs
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES warch_plan
 CATKIN_DEPENDS bspline_opt traj_utils 
#  DEPENDS system_lib
)

###########
## Build ##
###########
link_directories( ${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1 )
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)

include_directories(
  include 
  SYSTEM 
  third_party
  ${catkin_INCLUDE_DIRS} 
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(warch_plan_node
    src/plan_node.cpp
    src/plan_manager.cpp
    src/plan_vis.cpp
    src/corridor_generator.cpp
    src/a_star.cpp
    src/trajectory_generator.cpp
    src/bezier_base.cpp
    third_party/fast_methods/console/console.cpp
    third_party/fast_methods/fm/fmdata/fmcell.cpp
    third_party/fast_methods/ndgridmap/cell.cpp
)
# 使用其他文件里的msg文件时，需要添加add_dependencies
add_dependencies(warch_plan_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(warch_plan_node ${catkin_LIBRARIES} sdf_tools ${PCL_LIBRARIES} mosek64)

add_executable(traj_server 
  src/Bspline_traj_server.cpp
  # src/bezier_traj_server.cpp 
  # src/bezier_base.cpp
)
target_link_libraries(traj_server ${catkin_LIBRARIES})
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 3.0.2)
project(motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -O2 -Werror -Wall -Wno-unused -Wno-sign-compare)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        carla_msgs
        derived_object_msgs
        geometry_msgs
        sensor_msgs
        #planning_msgs
        #planning_srvs
        tf
        std_msgs
        visualization_msgs
        carla_waypoint_types
        reference_line
        common
        vehicle_state
        obstacle_manager
        collision_checker
        Eigen3
        #router_planner
        #cmpf_route_planner
        )

catkin_package(
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
set(CARLA_INCLUDE_DIRS /home/cuke/work/simulator/carla-0911s/carla/LibCarla/source)
#set(CARLA_RPC_INC_DIRS /home/cuke/work/simulator/carla-0911s/carla/Examples/CppClient/libcarla-install/include/system)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        src
        ${catkin_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
        ${CARLA_INCLUDE_DIRS}
        #${CARLA_RPC_INC_DIRS}
		/home/cuke/work/simulator/carla-0911s/carla/Build/boost-1.72.0-c8-install/include
		/home/cuke/work/simulator/carla-0911s/carla/Build/rpclib-v2.2.1_c3-c8-libstdcxx-install/include
		/home/cuke/work/simulator/carla-0911s/carla/Build/recast-cdce4e-c8-install/include
        ../router_planner/src/include
        ../lanelet2_map_server/include
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

set(planning_SRC
        src/frenet_lattice_planner/constraint_checker.cpp
        src/frenet_lattice_planner/end_condition_sampler.cpp
        src/frenet_lattice_planner/polynomial_trajectory_evaluator.cpp
        src/frenet_lattice_planner/lattice_trajectory1d.cpp
        src/frenet_lattice_planner/frenet_lattice_planner.cpp
        src/motion_planner.cpp
        src/planning_config.cpp
        src/motion_planner_node.cpp
        #src/route_planner_core.cpp
        src/reference_generator/reference_generator.cpp
    )

add_executable(motion_planner_node ${planning_SRC})


## Specify libraries to link a library or executable target against
set (CARLA_CLIENT_LIB /home/cuke/work/simulator/carla-0911s/carla/Examples/CppClient/libcarla-install/lib)

link_directories(
    $(CARLA_CLIENT_LIB)
)

target_link_libraries(motion_planner_node
        Eigen3::Eigen
        /usr/local/lib/libipopt.so.1
        "${CARLA_CLIENT_LIB}/libcarla_client.a"
        "${CARLA_CLIENT_LIB}/librpc.a"
        "${CARLA_CLIENT_LIB}/libboost_system.a"
        "${CARLA_CLIENT_LIB}/libDetour.a"
        "${CARLA_CLIENT_LIB}/libDetourCrowd.a"
        "${CARLA_CLIENT_LIB}/libDetourTileCache.a"
        #router_planner
        ${catkin_LIBRARIES}
        )


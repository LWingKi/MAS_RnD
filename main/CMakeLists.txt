CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(rnd)
find_package(orocos_kdl)
find_library(KDL_PARSER kdl_parser)
find_package(robif2b REQUIRED)
include(GNUInstallDirs)

set(MUJOCO_PATH "/home/wing/Documents/RND/mujoco-2.3.7")
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/bin)

find_package(glfw3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIRS})


set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

include_directories(${EIGEN3_INCLUDE_DIR})

#link all hearder in src
include_directories(
        ${PROJECT_SOURCE_DIR}/src
)
file(GLOB all_SRCS
        # "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.hpp"
        # "${PROJECT_SOURCE_DIR}/src/*.cpp"
        )


#add_executable(vereshchagin.cpp )
#TARGET_LINK_LIBRARIES(vereshchagin orocos-kdl)
add_library(gnu_plotter src/gnu_plotter.cpp)
target_link_libraries(gnu_plotter boost_filesystem boost_system boost_iostreams ${orocos_kdl_LIBRARIES} robif2b::kinova_gen3)

add_library(tf_utils src/tf_utils.cpp)
target_link_libraries(tf_utils ${orocos_kdl_LIBRARIES})

add_library(mujoco src/mujoco_env.cpp)
target_link_libraries(mujoco ${orocos_kdl_LIBRARIES} /home/wing/Documents/RND/mujoco-2.3.7/lib/libmujoco.so glfw)

add_library(logger src/logger.cpp)
target_link_libraries(logger ${orocos_kdl_LIBRARIES})

add_executable(main main.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(main logger gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

add_executable(robot_test robot_testing.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(robot_test logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

# add_executable(state_machine robot_testing_state_machine.cpp ${all_SRCS} )
# TARGET_LINK_LIBRARIES(state_machine logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 
# TARGET_LINK_LIBRARIES(main kdl_parser)
# TARGET_LINK_LIBRARIES(main orocos_kdl boost_filesystem boost_system boost_iostreams ) 

add_executable(elbow_rest elbow_rest.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(elbow_rest logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

add_executable(us2 us2.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(us2 logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

add_executable(us2_contacless us2_copy.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(us2_contacless logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

add_executable(us1 us1.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(us1 logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

add_executable(mujoco_test mujoco_test.cpp)
TARGET_LINK_LIBRARIES(mujoco_test mujoco)

add_executable(sos sos.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(sos logger tf_utils gnu_plotter ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 

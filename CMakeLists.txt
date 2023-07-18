CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(rnd)
find_package(orocos_kdl)
find_library(KDL_PARSER kdl_parser)
find_package(robif2b REQUIRED)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

include_directories(${EIGEN3_INCLUDE_DIR})



#add sub directory
# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../orocos_kinematics_dynamics/orocos_kdl" "${CMAKE_CURRENT_BINARY_DIR}/orocos_kinematics_dynamics/orocos_kdl/build")
# include_directories("$(CMAKE_CURRENT_SOURCE_DIR)/../orocos_kinematics_dynamics/orocos_kdl/src/")

# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../kdl_parser" "${CMAKE_CURRENT_SOURCE_DIR}/../kdl_parser/build")
# include_directories("$(CMAKE_CURRENT_SOURCE_DIR)/../kdl_parser/include/")

#link all hearder in src
include_directories(
        ${PROJECT_SOURCE_DIR}/src
)
file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        )


#add_executable(vereshchagin.cpp )
#TARGET_LINK_LIBRARIES(vereshchagin orocos-kdl)
add_library(gnu_plotter src/gnu_plotter.cpp)
target_link_libraries(gnu_plotter boost_filesystem boost_system boost_iostreams ${orocos_kdl_LIBRARIES})

add_executable(main main.cpp ${all_SRCS} )
TARGET_LINK_LIBRARIES(main ${orocos_kdl_LIBRARIES} boost_filesystem boost_system boost_iostreams ${KDL_PARSER} robif2b::kinova_gen3) 
# TARGET_LINK_LIBRARIES(main kdl_parser)
# TARGET_LINK_LIBRARIES(main orocos_kdl boost_filesystem boost_system boost_iostreams ) 

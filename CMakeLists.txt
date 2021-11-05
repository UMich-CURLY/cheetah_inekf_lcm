cmake_minimum_required(VERSION 2.8.3)

project(cheetah_inekf_lcm)
# add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# add_compile_options(-O3)
# add_compile_options(-DEIGEN_NO_DEBUG)
# add_compile_options(-march=native)
#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed -DBOOST_LOG_DYN_LINK -lboost_log -lpthread") #-DBOOST_LOG_DYN_LINK -lboost_log -lpthread needed for boost::log
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
SET(CMAKE_C_COMPILER /usr/bin/gcc)

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# InEKF 
find_package(inekf REQUIRED)
include_directories(${inekf_INCLUDE_DIRS})
message("inekf_INCLUDE_DIR: " ${inekf_INCLUDE_DIRS})
link_directories(${inekf_LIBRARY_DIRS})
message("inekf_LIBRARY_DIRS: " ${inekf_LIBRARY_DIRS})

# Boost
find_package(Boost 1.58 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIRS})

# LCM
# find_package(lcm REQUIRED)
# if (LCM_FOUND)
#     include_directories(${LCM_INCLUDE_DIRS})
# endif (LCM_FOUND)
find_library(LCM_LIBRARY lcm /usr/local/include/lcm/)
message("LCM_INCLUDE_DIR: " ${LCM_LIBRARY})

# Yaml:
# find_package(  yaml-cpp REQUIRED )
# include_directories(${YAML_CPP_INCLUDE_DIRS})
find_library(Yaml_cpp_LIBRARY yaml-cpp /usr/local/include/yaml-cpp/)
message("YAML_CPP_INCLUDE_DIR: " ${Yaml_cpp_LIBRARY})

if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

# find the yaml-cpp include directory
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
          PATH_SUFFIXES include
          PATHS
          ~/Library/Frameworks/yaml-cpp/include/
          /Library/Frameworks/yaml-cpp/include/
          /usr/local/include/
          /usr/include/
          /sw/yaml-cpp/         # Fink
          /opt/local/yaml-cpp/  # DarwinPorts
          /opt/csw/yaml-cpp/    # Blastwave
          /opt/yaml-cpp/
          ${YAMLCPP_DIR}/include/)

# find the yaml-cpp library
find_library(YAMLCPP_LIBRARY
             NAMES ${YAMLCPP_STATIC} yaml-cpp
             PATH_SUFFIXES lib64 lib
             PATHS ~/Library/Frameworks
                    /Library/Frameworks
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${YAMLCPP_DIR}/lib)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAMLCPP DEFAULT_MSG YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
mark_as_advanced(YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/kin
)

## Declare a C++ library
file(GLOB kinematics_src
    "${PROJECT_SOURCE_DIR}/src/kin/*.cpp"
)

add_library(cheetah_lcm_handler "${PROJECT_SOURCE_DIR}/src/communication/lcm_handler.cpp")

add_library(cheetah_measurement "${PROJECT_SOURCE_DIR}/src/utils/measurement.cpp")

add_library(cheetah_state "${PROJECT_SOURCE_DIR}/src/system/cheetah_state.cpp")

add_library(cheetah_system "${PROJECT_SOURCE_DIR}/src/system/cheetah_system.cpp")

add_library(body_estimator "${PROJECT_SOURCE_DIR}/src/estimator/body_estimator.cpp")

add_library(utils "${PROJECT_SOURCE_DIR}/src/utils/utils.cpp")

add_library(cheetah_kinematics_lcm ${kinematics_src})


add_executable(cheetah_estimator "${PROJECT_SOURCE_DIR}/src/cheetah_estimator.cpp" )
target_link_libraries(cheetah_estimator
    ${inekf_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Yaml_cpp_LIBRARY}
    # yaml-cpp
    ${LCM_LIBRARY}
    cheetah_measurement
    utils
    cheetah_kinematics_lcm
    cheetah_lcm_handler
    cheetah_system
    cheetah_state
    body_estimator
    cheetah_measurement
)

target_link_libraries(cheetah_state
    ${inekf_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Yaml_cpp_LIBRARY}
    # yaml-cpp
    cheetah_kinematics_lcm
    cheetah_measurement
    utils
)

target_link_libraries(body_estimator
    ${inekf_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Yaml_cpp_LIBRARY}
    # yaml-cpp
    cheetah_measurement
    utils
)

target_link_libraries(cheetah_system
    ${inekf_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Yaml_cpp_LIBRARY}
    # yaml-cpp
    cheetah_kinematics_lcm
    cheetah_measurement
    utils
)

target_link_libraries(cheetah_lcm_handler
    ${Boost_LIBRARIES}
    ${Yaml_cpp_LIBRARY}
    ${LCM_LIBRARY}
    # yaml-cpp
    cheetah_kinematics_lcm
    cheetah_measurement
    utils
)
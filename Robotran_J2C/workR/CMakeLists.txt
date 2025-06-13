#
#   Universite catholique de Louvain
#   Mechatronic, Electrical Energy, and Dynamic systems (iMMC/MEED) 
#   Robotran - MBsysC
#   https://www.robotran.be
#   Contact : info@robotran.be
#
#
# CMake for compiling a simple robotran project in C
#
# For us:
# cmake .. -DCMAKE_PREFIX_PATH="~/.robotran/mbsysc/MBsysC/install"
#
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       PROJECT MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


# CMake minimum version
cmake_minimum_required(VERSION 3.11)
set(CMAKE_BUILD_TYPE Debug)

# project name
# MODIFIED /!\
project (robotran_J2C C CXX)
# ADDED /!\
find_package(Threads REQUIRED)


# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

find_package(mbsysc 1.28 COMPONENTS module load utilities numerics realtime)

# Error message if mbsysc is not found, it is required
if(NOT EXISTS ${MBSYSC_DIRECTORY})
    message("The given path to MBsysC install directory was not found for this project.\n")
    message("(1) Either the given path(s) is/are not correct(s).\n")
    message("(2) Either the version number found is not correct.\n")
    message("\t [HINT] Please specify the path to the correct version when invoking 'cmake' by adding the option '-DCMAKE_PREFIX_PATH=PATH_TO_MBSYSC_INSTALLED/'\n")
    message("--> See above messages \n")
    message(FATAL_ERROR "mbsysc package is not found ! ")
else()
    message("Using MBsysC ${MBSYSC_VERSION} from ${MBSYSC_DIRECTORY}")
endif()

add_definitions( "-DMBSYSC_PATH=\"${MBSYSC_DIRECTORY}\"")

# Change default installation directory in case of not specified by user
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "default install path" FORCE)
endif()

include_directories(${MBSYSC_INCLUDE_DIRECTORIES})

# Include directories
include_directories("../symbolicR")
include_directories("../userfctR")
include_directories("src")

# Collect sources
file(GLOB SYMBOLIC_SRC "../symbolicR/*.c")
file(GLOB_RECURSE USER_SRC "../userfctR/*.c")
file(GLOB MAIN_SRC "src/*.c")

add_compile_definitions("BUILD_PATH=\"${CMAKE_CURRENT_BINARY_DIR}\"")
add_compile_definitions("PROJECT_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Generate the libraries
add_library(Project_user SHARED ${USER_SRC} ${MAIN_SRC}) 
add_library(Project_symbolic SHARED ${SYMBOLIC_SRC})

# Link libraries
target_link_libraries(Project_user ${MBSYSC_LIBRARIES})
target_link_libraries(Project_symbolic ${MBSYSC_LIBRARIES})
target_link_libraries(Project_user Project_symbolic ${MBSYSC_LIBRARIES})

# Generate the executable
add_executable(exe_${PROJECT_NAME} ${MAIN_SRC})
add_dependencies(exe_${PROJECT_NAME} Project_user Project_symbolic)
target_link_libraries(exe_${PROJECT_NAME} ${MBSYSC_LIBRARIES})
target_link_libraries(exe_${PROJECT_NAME} Project_user)
# ADDED /!\
target_link_libraries(exe_${PROJECT_NAME} Threads::Threads)

# Windows: default start up project is the executable
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "${exe_${PROJECT_NAME}}")

# Installation process
install(TARGETS Project_user DESTINATION .)
install(TARGETS Project_symbolic DESTINATION .)
install(TARGETS exe_${PROJECT_NAME} DESTINATION .)

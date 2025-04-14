cmake_minimum_required(VERSION 3.3.2)
project(arena_core)

# Enable testing via command-line flag (e.g. -DENABLE_TESTING=ON)
option(ENABLE_TESTING "Enable tests" OFF)
message("ALLOOOOO")

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
find_package(Pagmo REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)

find_package(fcl QUIET)
if (fcl_FOUND)
    # convert cmake target to variables for catkin_package DEPENDS
    if(TARGET ${FCL_LIBRARIES})
        get_target_property(LIBFCL_LOCATION ${FCL_LIBRARIES} LOCATION)
        get_target_property(LIBFCL_INTERFACE_LINK_LIBRARIES ${FCL_LIBRARIES} INTERFACE_LINK_LIBRARIES)
        get_target_property(LIBFCL_INCLUDE_DIRS ${FCL_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
        set(LIBFCL_LIBRARIES ${LIBFCL_LOCATION} ${LIBFCL_INTERFACE_LINK_LIBRARIES})
    else()
        set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
        set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
    endif()
    else()
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(LIBFCL_PC REQUIRED fcl)
        set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
        # find *absolute* paths to LIBFCL_LIBRARIES
        set(LIBFCL_LIBRARIES)
    foreach(_lib ${LIBFCL_PC_LIBRARIES})
        find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
        list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
    endforeach()
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
set(ENV{PKG_CONFIG_PATH} "/usr/lib/x86_64-linux-gnu/pkgconfig:$ENV{PKG_CONFIG_PATH}")

include_directories(
    libs/arena_core/include
    libs/arena_core/tests/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_subdirectory(libs)

message("SALUUUUUUUUUUUUUUUUUUUUUUT")
add_subdirectory(nodes)

# Optional testing
if(ENABLE_TESTING)
    message(STATUS "Building with tests enabled")
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    enable_testing()
endif()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

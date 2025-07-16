# Optional testing
if(ENABLE_TESTING)

    message(STATUS "--------------------------------------------Building with tests enabled")
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    enable_testing()

    ament_add_gtest(tests_nurbs tests_nurbs.cpp)
    target_link_libraries(tests_nurbs ${PROJECT_NAME}_arena_core)
    
endif()

if(ENABLE_COVERAGE)
    message(STATUS "Compiling with coverage flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# Library sources
set(ARENA_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/octomap_state_validity_checker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping/costmap_3D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planning/OMPLPlanner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math/algorithm/adaptive_voting_algorithm.cpp
)

# Library public headers
set(PUBLIC_HDRS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(ARENA_CORE_PUBLIC_HDRS
)

add_library(${PROJECT_NAME}_arena_core STATIC ${ARENA_CORE_SRCS})

target_include_directories(${PROJECT_NAME}_arena_core
    PUBLIC "$<BUILD_INTERFACE:${PUBLIC_HDRS_DIR}>"
           $<INSTALL_INTERFACE:include>
)

#add_dependencies(${PROJECT_NAME}_arena_core
#    ${catkin_EXPORTED_TARGETS}
#)

set(ARENA_CORE_PACKAGES_LIBRARIES
    ${OCTOMAP_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${LIBFCL_LIBRARIES}
    yaml-cpp
    Pagmo::pagmo
)

# Set OpenMP flags if OpenMP is found
if(OpenMP_CXX_FOUND)
    target_compile_options(${PROJECT_NAME}_arena_core 
    PRIVATE ${OpenMP_CXX_FLAGS}
    )
    set(ARENA_CORE_PACKAGES_LIBRARIES
        ${ARENA_CORE_PACKAGES_LIBRARIES} 
        OpenMP::OpenMP_CXX
    )
endif()

target_link_libraries(${PROJECT_NAME}_arena_core
    ${ARENA_CORE_PACKAGES_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_arena_core PROPERTIES
    PUBLIC_HEADER "${ARENA_CORE_PUBLIC_HDRS}"
)

install(TARGETS ${PROJECT_NAME}_arena_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/arena_core
)

install(DIRECTORY ${PUBLIC_HDRS_DIR}/arena_core
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()

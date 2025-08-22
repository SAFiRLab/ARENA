if (ENABLE_ROS2)
    set(ARENA_DEMOS_SRCS
        ${ARENA_DEMOS_SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_problem.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_nurbs_analyzer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/nurbs_analyzer_old.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/old_nurbs.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/costmap_3D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/costmap_mapping.cpp
    )

    add_subdirectory(nodes)
endif()

add_subdirectory(tests)

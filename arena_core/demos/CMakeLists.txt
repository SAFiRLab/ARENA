set(ARENA_DEMOS_SRCS
    ${ARENA_DEMOS_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_problem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_nurbs_analyzer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/nurbs_analyzer_old.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/old_nurbs.cpp
)

add_subdirectory(nodes)
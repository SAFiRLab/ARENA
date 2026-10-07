add_executable(pagmo_test pagmo_test.cpp ${ARENA_DEMOS_SRCS})
target_link_libraries(pagmo_test ${PROJECT_NAME}_arena_core)


install(TARGETS
  pagmo_test
  DESTINATION lib/${PROJECT_NAME}
)

if(ENABLE_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    enable_testing()

    ament_add_gtest(traversability_map_test
        husky/mapping/traversability_map_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/husky/mapping/traversability_map.cpp
        ${ARENA_DEMOS_SRCS}
    )
    ament_target_dependencies(traversability_map_test grid_map_core pcl_conversions)

    ament_add_gtest(map_generator_test
        linedrone/map_generator/map_generator_test.cpp
        ${ARENA_DEMOS_MAP_GENERATOR_SRCS}
    )
    target_link_libraries(map_generator_test ${OCTOMAP_LIBRARIES} yaml-cpp)
    ament_target_dependencies(map_generator_test grid_map_core)
endif()

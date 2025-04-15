#add_executable(costmap_3D_node costmap_3D_node.cpp)
#target_link_libraries(costmap_3D_node ${PROJECT_NAME}_arena_core)

add_executable(pagmo_test pagmo_test.cpp)
target_link_libraries(pagmo_test ${PROJECT_NAME}_arena_core)

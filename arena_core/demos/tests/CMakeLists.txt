add_executable(pagmo_test pagmo_test.cpp ${ARENA_DEMOS_SRCS})
target_link_libraries(pagmo_test ${PROJECT_NAME}_arena_core)


install(TARGETS
  pagmo_test
  DESTINATION lib/${PROJECT_NAME}
)

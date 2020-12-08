find_package(GTest REQUIRED)

# Common module tests
add_executable(${PROJECT_NAME}_common_tests test_common.cpp)
target_link_libraries(${PROJECT_NAME}_common_tests PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
target_cxx_version(${PROJECT_NAME}_common_tests PRIVATE VERSION 14)
add_gtest_discover_tests(${PROJECT_NAME}_common_tests)

# Plane Intersection tests
add_executable(${PROJECT_NAME}_plane_intersection_tests test_plane_intersection.cpp)
target_link_libraries(${PROJECT_NAME}_plane_intersection_tests PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
target_cxx_version(${PROJECT_NAME}_plane_intersection_tests PRIVATE VERSION 14)
add_gtest_discover_tests(${PROJECT_NAME}_plane_intersection_tests)

add_executable(${PROJECT_NAME}_advancing_front_tests test_advancing_front.cpp)
target_link_libraries(${PROJECT_NAME}_advancing_front_tests PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
target_cxx_version(${PROJECT_NAME}_advancing_front_tests PRIVATE VERSION 14)
add_gtest_discover_tests(${PROJECT_NAME}_advancing_front_tests)

# Install the test executables so they can be run independently later if needed
install(
  TARGETS
    ${PROJECT_NAME}_common_tests
    ${PROJECT_NAME}_plane_intersection_tests
    ${PROJECT_NAME}_advancing_front_tests
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests
)

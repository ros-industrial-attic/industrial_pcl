cmake_minimum_required(VERSION 3.1)
project(pcl_advancing_front VERSION 0.1.0 LANGUAGES CXX)

find_package(cmake_common_scripts REQUIRED)
find_package(PCL REQUIRED COMPONENTS common surface visualization io console)

# Advancing Front Library
add_library(${PROJECT_NAME} SHARED
  src/advancing_front.cpp
)
target_compile_options(${PROJECT_NAME} PUBLIC)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME} PRIVATE VERSION 14)

# Advancing Front Tool
add_executable(${PROJECT_NAME}_tool
  src/advancing_front_reconstruction.cpp
)
target_link_libraries(${PROJECT_NAME}_tool PUBLIC ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME} PRIVATE VERSION 14)

configure_package(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_tool
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

#############
## Testing ##
#############
if(INDUSTRIAL_PCL_BUILD_TESTS)
  enable_testing()
  add_run_tests_target()
  add_subdirectory(test)
endif()

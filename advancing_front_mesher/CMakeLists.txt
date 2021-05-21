cmake_minimum_required(VERSION 3.5.1)
project(advancing_front_mesher VERSION 0.1.0 LANGUAGES CXX)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(PCL REQUIRED COMPONENTS common surface visualization io console)

if(PCL_FOUND AND PCL_VERSION VERSION_LESS 1.9)
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9 COMPONENTS common surface visualization io console)
endif()

# Advancing Front Library
add_library(${PROJECT_NAME} SHARED
  src/advancing_front.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME} PRIVATE VERSION 14)
if(PCL_VERSION VERSION_LESS 1.10)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DPCL_LESS_1_10)
endif()

# Add OpenMP support, if available
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

  # Create the target if it doesn't exist
  if(NOT TARGET OpenMP::OpenMP_CXX)
      find_package(Threads REQUIRED)
      add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
      set_property(TARGET OpenMP::OpenMP_CXX
                   PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
      # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
      set_property(TARGET OpenMP::OpenMP_CXX
                   PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
  endif()

  # Link the library against the OpenMP target
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
else(OPENMP_FOUND)
  message (STATUS "OpenMP not found")
endif()

# Advancing Front Tool
add_executable(${PROJECT_NAME}_tool
  src/advancing_front_reconstruction.cpp
)
target_link_libraries(${PROJECT_NAME}_tool PUBLIC ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME} PRIVATE VERSION 14)

configure_package(
  NAMESPACE
    industrial_pcl
  TARGETS
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
  add_run_tests_target(INDUSTRIAL_PCL_RUN_TESTS)
  add_subdirectory(test)
endif()

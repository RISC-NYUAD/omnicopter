cmake_minimum_required(VERSION 3.8)
project(libkdtp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Required packages
find_package(ament_cmake REQUIRED)

# Include directories
include_directories(include)

# Add the library
add_library(${PROJECT_NAME} SHARED 
    src/dof.cc		
    src/spline.cc		
    src/localpath.cc	
    src/poly.cc
)

# Export include directories
ament_export_include_directories(include)

# Export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

# Install include files
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Final package
ament_package()

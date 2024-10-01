cmake_minimum_required(VERSION 3.9)
project(libkdtp)

catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} )
add_library(${PROJECT_NAME} SHARED 
	src/dof.cc		
	src/spline.cc		
	src/localpath.cc	
	src/poly.cc
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	)

install(TARGETS libkdtp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


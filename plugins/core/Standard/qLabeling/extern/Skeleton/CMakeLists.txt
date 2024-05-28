cmake_minimum_required( VERSION 3.10 )

project( QSKELETON_LIB )
 
add_library( ${PROJECT_NAME} STATIC )


# To remove very annoying "NO ARGUMENTS THAT DEPEND ON A TEMPLATE PARAMETER" errors with G++!
if( UNIX OR MINGW )
	target_compile_options( ${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fpermissive -w> )
elseif( MSVC )
	target_compile_options( ${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/bigobj> )
endif()

find_package(Eigen3 REQUIRED)
#find_package(nanoflann REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
#message(STATUS "nanoflann_INCLUDE_DIR: ${nanoflann_INCLUDE_DIR}")
   

FILE(GLOB Skelelon_HDR *.h *.hpp)
FILE(GLOB Skelelon_SRC *.cpp)

FILE(GLOB Skelelon_Graph_HDR graph/*.h graph/*.hpp)
FILE(GLOB Skelelon_Graph_SRC graph/*.cpp)

FILE(GLOB Skelelon_EigenTools_HDR eigen_tools/*.h eigen_tools/*.hpp)
FILE(GLOB Skelelon_EigenTools_SRC eigen_tools/*.cpp)
 
target_sources( ${PROJECT_NAME}
	PUBLIC
		 
	PRIVATE
	    ${Skelelon_HDR}
		${Skelelon_Graph_HDR}
		${Skelelon_EigenTools_HDR}
		${Skelelon_SRC}
		${Skelelon_Graph_SRC}
		${Skelelon_EigenTools_SRC}
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
	    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions( ${PROJECT_NAME}
	PRIVATE
		_CRT_SECURE_NO_DEPRECATE
		_CRT_SECURE_NO_WARNINGS
		NOMINMAX
)

source_group("graph\\Hearder Files" FILES ${Skelelon_Graph_HDR})
source_group("graph\\Source Files" FILES ${Skelelon_Graph_SRC})

source_group("eigen_tools\\Hearder Files" FILES ${Skelelon_EigenTools_HDR})
source_group("eigen_tools\\Source Files" FILES ${Skelelon_EigenTools_SRC})
 

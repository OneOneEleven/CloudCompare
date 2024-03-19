option( PLUGIN_STANDARD_QSKELETON "Install qSkeleton plugin" OFF )

if( PLUGIN_STANDARD_QSKELETON )
	project(QSKELETON_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} TYPE  standard)

	add_subdirectory( extern/Skeleton)
	#add_subdirectory( include )
	#add_subdirectory( src )
	#add_subdirectory( ui )
	
	FILE(GLOB Skelelon_HDR include/*.h)
    FILE(GLOB Skelelon_SRC src/*.cpp)
    FILE(GLOB Skelelon_UI ui/*.ui)
	set_target_properties(${PROJECT_NAME}
	   PROPERTIES
       RUNTIME_OUTPUT_DIRECTORY ${CLOUDCOMPARE_PLUGIN_DEST_FOLDER}
	)
 
	target_sources( ${PROJECT_NAME}
	    PRIVATE
		${Skelelon_HDR}
		${Skelelon_SRC}
		${Skelelon_UI}
	)

    target_include_directories(${PROJECT_NAME}
	    PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
	)
 
	target_link_libraries( ${PROJECT_NAME} QSKELETON_LIB )
endif()

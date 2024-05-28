option( PLUGIN_STANDARD_QLABELING "Install QLABELING plugin" OFF )

if( PLUGIN_STANDARD_QLABELING )
	project(QLABELING_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} TYPE  standard)

	#add_subdirectory( extern/Skeleton)
	#add_subdirectory( include )
	#add_subdirectory( src )
	#add_subdirectory( ui )
	
	FILE(GLOB Labeling_HDR include/*.h)
    FILE(GLOB Labeling_SRC src/*.cpp)
    FILE(GLOB Labeling_UI ui/*.ui)
	set_target_properties(${PROJECT_NAME}
	   PROPERTIES
       RUNTIME_OUTPUT_DIRECTORY ${CLOUDCOMPARE_PLUGIN_DEST_FOLDER}
	)
 
	target_sources( ${PROJECT_NAME}
	    PRIVATE
		${Labeling_HDR}
		${Labeling_SRC}
		${Labeling_UI}
	)

    target_include_directories(${PROJECT_NAME}
	    PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
	)
 
	#target_link_libraries( ${PROJECT_NAME} QLABELING_LIB )
endif()

include(FindPackageHandleStandardArgs)

# Look for the x264 inlcude directory using the hints.
FIND_PATH(X264_INCLUDE_DIR x264.h
	HINTS "${CMAKE_CURRENT_LIST_DIR}/include/"
	NO_DEFAULT_PATH
	)

# If this was not successfull check the default directory
IF(${X264_INCLUDE_DIR} STREQUAL X264_INCLUDE_DIR-NOTFOUND)
	FIND_PATH(X264_INCLUDE_DIR x264.h)
ENDIF()

# This is my last resort... The directory name may be different...
IF(${X264_INCLUDE_DIR} STREQUAL X264_INCLUDE_DIR-NOTFOUND)
	FILE(GLOB_RECURSE X264_INCLUDE_DIR "/x264*/x264.h")
	IF(NOT ${X264_INCLUDE_DIR} STREQUAL X264_INCLUDE_DIR-NOTFOUND)
		GET_FILENAME_COMPONENT(X264_INCLUDE_DIR ${X264_INCLUDE_DIR} PATH)
	ENDIF()
ENDIF()

# Check if x264 include directory was found.
IF(${X264_INCLUDE_DIR} STREQUAL X264_INCLUDE_DIR-NOTFOUND)
	MESSAGE(STATUS "Can't find x264 inlcude directory!")
ELSE()
	#MESSAGE(STATUS "Found x264 inlcude directory: ${X264_INCLUDE_DIR}")
	#MESSAGE(STATUS "Using x264 dir parent as hint: ${CMAKE_CURRENT_LIST_DIR}")
	
	# Look for the x264 libraray (libx264.lib)
	IF(NOT WIN32)
		FIND_LIBRARY(X264_LIBRARY x264
			HINTS "${CMAKE_CURRENT_LIST_DIR}/lib/"
			NO_DEFAULT_PATH)
		IF(${X264_LIBRARY} STREQUAL X264_LIBRARY-NOTFOUND)
			FIND_LIBRARY(X264_LIBRARY libx264
				HINTS "${CMAKE_CURRENT_LIST_DIR}/lib/"
				NO_DEFAULT_PATH)
		ENDIF()
	ELSE()
		FIND_FILE(X264_LIBRARY NAMES libx264.lib HINTS "${CMAKE_CURRENT_LIST_DIR}/lib/")
	ENDIF()

	# Check if both, the include directory and the libraray have been found.
	IF(NOT ${X264_INCLUDE_DIR} STREQUAL X264_INCLUDE_DIR-NOTFOUND
		AND NOT ${X264_LIBRARY} STREQUAL X264_LIBRARY-NOTFOUND)
		
		# Set the inlucde and lib directory.
		SET(X264_FOUND 1)
		SET(X264_INCLUDE_DIRS ${X264_INCLUDE_DIR})
		SET(X264_LIBRARIES ${X264_LIBRARY})
		SET(X264_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/")
	
		#MESSAGE(STATUS "x264 include dir: ${X264_INCLUDE_DIR}")
		#MESSAGE(STATUS "x264 lib: ${X264_LIB}")
		#MESSAGE(STATUS "x264 lib dir: ${X264_LIB_DIR}")
	ELSE()
		MESSAGE(STATUS "Can't find x264")
	ENDIF()
ENDIF()

cmake_minimum_required(VERSION 2.6)
 
project(ShipProject)
 
if(WIN32)
	set(CMAKE_MODULE_PATH "$ENV{OGRE_HOME}/CMake/;${CMAKE_MODULE_PATH}")
endif(WIN32)

set (
  3DEXP_CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}
  CACHE PATH "Custom module path"
)
 
if(UNIX)
	if(EXISTS "/usr/local/lib/OGRE/cmake")
	  set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
	elseif(EXISTS "/usr/lib/OGRE/cmake")
	  set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
	else ()
    #message(SEND_ERROR "Failed to find module path.")
    set(CMAKE_MODULE_PATH "${3DEXP_CMAKE_MODULE_PATH};${CMAKE_MODULE_PATH}")
	endif(EXISTS "/usr/local/lib/OGRE")
endif(UNIX)
 
IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 

set(OGRE_STATIC TRUE CACHE BOOL "Forcing static build" FORCE)
set(OGRE_ARCHIVE_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
set(OGRE_LIBRARY_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
set(OGRE_RUNTIME_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
add_subdirectory(Ogre)

# Find Boost
if (NOT OGRE_BUILD_PLATFORM_IPHONE)
	if (WIN32 OR APPLE)
		set(Boost_USE_STATIC_LIBS TRUE)
	else ()
		# Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
		set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
	endif ()
	if (MINGW)
		# this is probably a bug in CMake: the boost find module tries to look for
		# boost libraries with name libboost_*, but CMake already prefixes library
		# search names with "lib". This is the workaround.
		set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
	endif ()
	set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
	# Components that need linking (NB does not include header-only components like bind)
	set(OGRE_BOOST_COMPONENTS thread date_time)
	find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
	if (NOT Boost_FOUND)
		# Try again with the other type of libs
		set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
		find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
	endif()
	find_package(Boost QUIET)
 
	# Set up referencing of Boost
	include_directories(${Boost_INCLUDE_DIR})
	add_definitions(-DBOOST_ALL_NO_LIB)
	set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
endif()

# Bit hacky way so all the output files are placed on a specified folder.
# Code found on http://stackoverflow.com/questions/7747857/in-cmake-how-do-i-work-around-the-debug-and-release-directories-visual-studio-2
# First for the generic no-config case (e.g. with mingw)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
# Second, for multi-config builds (e.g. msvc)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/bin )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/bin )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/bin )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

if (MSVC)
    set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_CXX_FLAGS_MINSIZEREL
        )
    foreach(CompilerFlag ${CompilerFlags})
        string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
    endforeach()
ENDIF(MSVC)

if (UNIX OR MINGW)
  # se for o g++ para *nix ou o MinGW para Windows:
  add_definitions(-Wall -ansi -U_FORTIFY_SOURCE -std=gnu++0x)
  if(NOT ANDROID)
    add_definitions(-msse2)
  endif()
endif ()

# PROJECT SOURCES
set(SRCS
	./src/BaseApplication.h
    ./src/BaseApplication.cpp
	./src/TutorialApplication.h
	./src/TutorialApplication.cpp
    ./src/gameobject.h
    ./src/gameobject.cc
    ./src/gamecamera.h
    ./src/gamecamera.cc
    ./src/physicsmanager.h
    ./src/physicsmanager.cc
)
include_directories ("src")

add_subdirectory(BulletPhysics)
add_subdirectory(btogre)

include_directories( ${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${BTOGRE_INCLUDE_DIRS}
)
 
add_executable(ShipProject ${SRCS})
 
target_link_libraries(ShipProject ${OGRE_LIBRARIES} 
                                  ${OIS_LIBRARIES} 
                                  ${BULLET_LIBRARIES}
                                  ${BTOGRE_LIBRARIES})
 

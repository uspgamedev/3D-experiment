
cmake_minimum_required(VERSION 2.6)
 
project(ShipProject)
 
 
IF (NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)

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

add_subdirectory(externals)

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

add_executable(ShipProject ${SRCS})
target_include_directories(ShipProject 
    PRIVATE "src"
)
 
target_link_libraries(ShipProject BtOgre)
 

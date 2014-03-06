cmake_minimum_required(VERSION 2.8.0)

project(BtOgre)

find_package(OGRE REQUIRED)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include/
    ${BULLET_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
)

add_library(BtOgre STATIC BtOgre.cpp)
target_link_libraries(BtOgre ${BULLET_LIBRARIES} ${OGRE_LIBRARIES})

list (APPEND BTOGRE_LIBRARIES BtOgre)
set(BTOGRE_LIBRARIES ${BTOGRE_LIBRARIES} PARENT_SCOPE)
set(BTOGRE_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/ PARENT_SCOPE)
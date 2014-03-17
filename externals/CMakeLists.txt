
set(OGRE_STATIC TRUE CACHE BOOL "Forcing static build" FORCE)
set(OGRE_ARCHIVE_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
set(OGRE_LIBRARY_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
set(OGRE_RUNTIME_OUTPUT ${CMAKE_SOURCE_DIR}/bin)
add_subdirectory(Ogre) #generates 'all utility' target OgreAll

add_subdirectory(BulletPhysics) #generate 'all utility' target BulletAll

add_subdirectory(btogre) #generates target BtOgre



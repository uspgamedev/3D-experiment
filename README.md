3D-experiment
=============

Proof of concept for a 3D game engine using Ogre (v1.9 forward) and Bullet (v2.82 forward, included).



-------------------------------
 Compilation Notes
-------------------------------
-Tested in Win7/VS2012, precompiled Ogre SDK
-Tested in Linux, built Ogre SDK from source

BUILDING OGRE:
Ogre uses several external libraries, and it includes some of them.
However, while Ogre's CMake will complain if some are not available (like Boost),
it will say nothing about others and then when you try to use Ogre some (important)
stuff might not work.

Libs we noticed that you might have missed:
-libzzip: without it, Ogre will be uncapable of loading resources from .zip files,
    which we use. Might prevent game from loading at all, and log shows exception
    about not being able to load a .zip file.
-FreeImage: without it, Ogre will be uncapable of loading pretty much any texture file.
    And any you try to render will show up as a striped black/yellow pattern. Ogre log
    will show exceptions about not finding a codec for the texture extension you've tried.
    
If any of this happened, you will need to install the lib, re-generate Ogre's CMake
and rebuild/reinstall Ogre, and afterwards rebuild your app.

Prebuilt SDK (at this time, only available for Visual Studio) comes with everything but samples
compiled, and all required external dependencies either included or already compiled together with
Ogre's libraries and plugins.
Ogre source may be downloaded from Ogre site or most likely as a zip file from their repository,
remember we're using v1.9!

BUILDING BULLET:
We've included a Bullet Physics engine build along with this project, and it should not
cause any problems when compiling.

However, Bullet's CMake option "USE_DOUBLE_PRECISION" is known to cause linking errors if ON.
We could alter our Bullet build to fix this, but haven't so far.
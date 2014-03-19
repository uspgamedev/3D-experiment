3D-experiment
=============

Proof of concept for a 3D game engine using Ogre (v1.9 forward, included) and Bullet (v2.82 forward, included).

-------------------------------
 RUNNING THE PROGRAM
-------------------------------
Project's <>/bin/media folder wasn't included in the repository (so far), you'll need to download
it from somewhere else. Ask the devs!



-------------------------------
 Compilation Notes
-------------------------------
-Tested in Win7/VS2013 and Linux

BUILDING THE PROJECT (3D-experiment/ShipProject)
On Visual Studio: after generating VS projects, you'll need to set the ShipProject as start-up project,
and working directory as the <project folder>/bin folder - this is where the executable is placed, so you
can use the macro $(TargetDir) as well.
You might need to fix the cg.dll directory from <>/bin/?/cg.dll to <>/bin/cg.dll after running cmake/compiling.

BUILDING BULLET:
We've included a Bullet Physics (v2.82) engine build along with this project, and it should not
cause any problems when compiling.
However, Bullet's CMake option "USE_DOUBLE_PRECISION" is known to cause linking errors if ON.

BUILDING OGRE:
We've also included a Ogre build (v1.9.0) along with this project, in order to compile and link Ogre statically.
Ogre uses several external libraries, however, while Ogre's CMake will complain if some are not available (like Boost),
it will say nothing about others and then when you try to use Ogre some (important)
stuff might not work.

In Windows/VS2013, it is likely that you do not have Ogre's dependencies installed, in this case it's best
to download our Ogre dependencies pack and unzip it at <>/Ogre/, which should create a <>/Ogre/Dependencies folder,
and they'll be statically compiled along with the project.

In Linux, it's WAY easier to just install the lib dev packages from the repository (apt-get/aptitude install, etc).
You may be able to compile them together as well, but we didn't test this (so far).

These Ogre dependencies are:
* Cg (3.1) [Optional - required for CgProgramManager plugin (shader stuff)]
* FreeImage (3.15.3)
* freetype (2.4.9)
* OIS (1.4) [Optional - for input]
* zlib (1.2.8)
* zziplib (0.13.62) 
* Boost (1.55) - thread and date_time compiled [Optional - required for some ogre multithreading support]

Note that when getting the dependencies pack to compile them, we do not compile 'cg', it is already compiled (for Windows) in the pack.
You might need to manually set some of the OgreDependencies projects to use the static runtime (project properties -> C++ -> code generation; /MT or /MTd for debug)

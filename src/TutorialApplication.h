/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"
#include <vector>
#include <gameobject.h>

class TutorialApplication : public BaseApplication
{
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);

protected:
    virtual void createScene(void);
	virtual void destroyScene();
	virtual void createCamera(void);
	virtual void createViewports(void);
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed( const OIS::KeyEvent &arg );
    virtual bool keyReleased( const OIS::KeyEvent &arg );
	virtual bool mouseMoved( const OIS::MouseEvent &arg );
	virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

	void createPlane(const std::string& name, const Ogre::Vector3& dir, double dist, double width=20.0, double height=20.0);
	void createSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16);

	ShipProject::GameObject* player;
	double mRotate;          // The rotate constant
	double mMove;            // The movement constant
	Ogre::Vector3 mDirection;     // Value to move in the correct direction

	Ogre::InstanceManager* balls;

    std::vector<ShipProject::GameObject*> objects_;
};

#endif // #ifndef __TutorialApplication_h_

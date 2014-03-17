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
#include <list>
#include <map>
#include <gameobject.h>
#include <gamecamera.h>

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
    virtual bool buttonPressed( const OIS::JoyStickEvent &arg, int button );
	virtual bool buttonReleased( const OIS::JoyStickEvent &arg, int button );
	virtual bool axisMoved( const OIS::JoyStickEvent &arg, int axis );
    virtual bool sliderMoved( const OIS::JoyStickEvent &arg, int index);

	void createPlane(const std::string& name, const Ogre::Vector3& dir, double dist, double width=20.0, double height=20.0);
	void createSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16);
    void shoot();
    void evilShot(ShipProject::GameObject* enemy, double speed);
    void summonEvilBall();
    void handleCollisions(ShipProject::GameObject* obj1, ShipProject::GameObject* obj2, btManifoldPoint& pt);

    ShipProject::GameCamera* camera;
	ShipProject::GameObject* player;
	double mRotate;          // The rotate constant
	double mMove;            // The movement constant
	Ogre::Vector3 mDirection;     // Value to move in the correct direction

	Ogre::InstanceManager* balls;
	Ogre::ParticleSystem* explosions_;

    std::list<ShipProject::GameObject*> objects_;

    struct Projectile {
        ShipProject::GameObject* owner;
        double lifetime;

        Projectile() {};
        Projectile(ShipProject::GameObject* _owner, double _lifetime) 
            : owner(_owner), lifetime(_lifetime) {}
    };
    std::map<std::string, Projectile> projectiles_;
    std::list<std::string> projs_to_remove_;
    
    struct Enemy {
        ShipProject::GameObject* owner;
        Ogre::ParticleSystem* particles;
        double cooldown;
        double timeElapsed;

        Enemy() {}
        Enemy(ShipProject::GameObject* _owner, double _cooldown) 
            : owner(_owner), particles(nullptr), cooldown(_cooldown), timeElapsed(0) {}
    };
    std::map<std::string, Enemy> enemies_;
    std::list<std::string> enemies_to_remove_;
};

#endif // #ifndef __TutorialApplication_h_

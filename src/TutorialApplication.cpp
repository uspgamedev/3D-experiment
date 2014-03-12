/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
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
#include "TutorialApplication.h"

#include <physicsmanager.h>
#include <btBulletDynamicsCommon.h>

#include <BtOgreExtras.h>
#include <BtOgreGP.h>

#include <functional>
#include <string>
#include <algorithm>

#define PLAYER_RADIUS 0.7
#define BALL_RADIUS 0.5
#define PROJECTILE_RADIUS 0.1
#define AREA_RANGE 20.0

#ifdef _WIN32
#define MOVE_FORWARD_AXIS   1
#define MOVE_SIDEWAYS_AXIS  2
#define CAMERA_YAW_VALUE    joy.mAxes[0].abs
#define CAMERA_PITCH_VALUE  joy.mSliders[0].abX
#else
#define MOVE_FORWARD_AXIS   1
#define MOVE_SIDEWAYS_AXIS  0
#define CAMERA_YAW_VALUE    joy.mAxes[2].abs
#define CAMERA_PITCH_VALUE  joy.mAxes[3].abs
#endif

using ShipProject::PhysicsManager;
using ShipProject::GameObject;
using ShipProject::CollisionGroup;
using ShipProject::CollisionLogic;

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void) {
	mRotate = 0.05;
	mMove = 250;
	mDirection = Ogre::Vector3::ZERO;
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void) {
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void) {
	PhysicsManager::reference()->Initialize(btVector3(0,0,0), mSceneMgr);

    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    //mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
 
    createSphere("playerBall", PLAYER_RADIUS);
    createSphere("evilBall", BALL_RADIUS);
    createSphere("projectile", PROJECTILE_RADIUS);
    Ogre::MaterialPtr mMat = Ogre::MaterialManager::getSingleton().create("Projectile/Red", "General", true);
    mMat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(1.0, 0.1, 0));
    mMat->getTechnique(0)->getPass(0)->setEmissive(Ogre::ColourValue(1.0, 0.1, 0));
    Ogre::MaterialPtr mMat2 = Ogre::MaterialManager::getSingleton().create("Projectile/Blue", "General", true);
    mMat2->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0, 0.1, 1.0));
    mMat2->getTechnique(0)->getPass(0)->setEmissive(Ogre::ColourValue(0, 0.1, 1.0));
    Ogre::MaterialPtr mMat3 = Ogre::MaterialManager::getSingleton().getByName("Ogre/Skin")->clone("PlayerBall");
    mMat3->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0.1,0.1,1.0,0.5));
    mMat3->setSceneBlending(Ogre::SceneBlendType::SBT_TRANSPARENT_ALPHA);

    Ogre::Entity* playerEnt = mSceneMgr->createEntity("Player", "playerBall");
    playerEnt->setCastShadows(true);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(playerEnt);
    playerEnt->setMaterialName("PlayerBall");
    player = new GameObject(playerEnt, 40);
    player->SetupCollision(new btSphereShape(PLAYER_RADIUS), CollisionGroup::PLAYER, 
                            CollisionGroup::BALLS | CollisionGroup::PROJECTILES_BALLS | CollisionGroup::WALLS);
    player->body()->setAngularFactor(btVector3(0.0, 0.0,0.0));
    objects_.push_back( player );

    camera->SetParameters(Ogre::Vector3(0, PLAYER_RADIUS/2, 0), PLAYER_RADIUS*5);
    camera->SetDistance(PLAYER_RADIUS*5);
    camera->AttachTo(player);
 
	this->createPlane("ground", Ogre::Vector3::UNIT_Y, -(AREA_RANGE/2));
	this->createPlane("ceiling", -Ogre::Vector3::UNIT_Y, -(AREA_RANGE/2));
	this->createPlane("leftwall", Ogre::Vector3::UNIT_X, -(AREA_RANGE/2));
	this->createPlane("rightwall", -Ogre::Vector3::UNIT_X, -(AREA_RANGE/2));
	this->createPlane("backwall", Ogre::Vector3::UNIT_Z, -(AREA_RANGE/2));
    this->createPlane("frontwall", -Ogre::Vector3::UNIT_Z, -(AREA_RANGE/2));

	// LIGHTS
	Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setPosition(Ogre::Vector3(0, 0, 0));
    pointLight->setDiffuseColour(1.0, 1.0, 1.0);
    pointLight->setSpecularColour(1.0, 1.0, 1.0);
 
    Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(0, 0, 0.25));
    directionalLight->setSpecularColour(Ogre::ColourValue(0, 0, 0.25));
    directionalLight->setDirection(Ogre::Vector3( 0, -1, 1 )); 

    CollisionLogic logic = 
        [this](GameObject* obj1, GameObject* obj2, btManifoldPoint& pt) { 
            this->handleCollisions(obj1,obj2,pt); 
    };
    PhysicsManager::reference()->set_collision_logic(logic);

	// GOTTA CHECK IF WE SUPPORT THE INSTANCING TECHNIQUE (involves hardware and software)
	
	//balls = mSceneMgr->createInstanceManager("balls", "mySphereMesh", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME, 
	//	Ogre::InstanceManager::InstancingTechnique::HWInstancingVTF, 100, Ogre::IM_USEALL);
}
void TutorialApplication::destroyScene() {
    for (auto obj : objects_) {
        delete obj;
    }
    objects_.clear();
    delete camera;
	delete PhysicsManager::reference();
}

void TutorialApplication::createCamera(void)
{
    camera = new ShipProject::GameCamera(mSceneMgr);
    // create the camera
    mCamera = camera->camera(); 
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
void TutorialApplication::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));    
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	if (BaseApplication::frameRenderingQueued(evt)) {
		PhysicsManager::reference()->Update(evt.timeSinceLastFrame);
        
		//player->Translate( player->entity()->getParentSceneNode()->getOrientation() * mDirection * evt.timeSinceLastFrame);
        player->Move( camera->actual_orientation() * mDirection * (1.0/50.0) );
        
        if (mJoyStick) {
            /* For some reason, OIS reports one of my gamepad's axes as a 'slider'(at least in windows)...
            Also, joystick event are called only when axis/slider value changes, and since they aren't 
            continuous, you can stick a axis to one side continuosly but the event will only be called once,
            so we need to do this here.
            Finally, aparently after moving from the initial state of value 0 the axis rest at [-]256 and not at 0...
            */
            OIS::JoyStickState joy = mJoyStick->getJoyStickState();
            double yaw = 0;
            if ( Ogre::Math::Abs(CAMERA_YAW_VALUE) > 256)
                yaw = -150 * evt.timeSinceLastFrame * CAMERA_YAW_VALUE / OIS::JoyStick::MAX_AXIS;
            double pitch = 0;
            if ( Ogre::Math::Abs(CAMERA_PITCH_VALUE) > 256)
                pitch = -150 * evt.timeSinceLastFrame * CAMERA_PITCH_VALUE / OIS::JoyStick::MAX_AXIS;
            camera->Rotate(yaw, pitch);
            //player->Rotate(pitch*100, yaw*100, 0);

            if (joy.mPOV[0].direction == OIS::Pov::North)
                camera->SetDistance( camera->GetDistance() - evt.timeSinceLastFrame*2);
            else if (joy.mPOV[0].direction == OIS::Pov::South)
                camera->SetDistance( camera->GetDistance() + evt.timeSinceLastFrame*2);

        }

        for (auto& pair: projectiles_) {
            Projectile& proj = pair.second;
            proj.lifetime -= evt.timeSinceLastFrame;
            if (proj.lifetime < 0) {
                //objects_.remove(proj.owner);
                if (std::find(projs_to_remove_.begin(), projs_to_remove_.end(), pair.first) == projs_to_remove_.end() )
                    projs_to_remove_.push_back(pair.first);
                //delete proj.owner;
            }
        }
        for (auto& name : projs_to_remove_) {
            Projectile& proj = projectiles_[name];
            objects_.remove(proj.owner);
            delete proj.owner;
            projectiles_.erase(name);
        }
        projs_to_remove_.clear();
        for (auto& name : enemies_to_remove_) {
            Enemy& ene = enemies_[name];
            objects_.remove(ene.owner);
            delete ene.owner;
            enemies_.erase(name);
        }
        enemies_to_remove_.clear();        
            
        for (auto& pair: enemies_) {
            Enemy& ene = pair.second;
            ene.timeElapsed += evt.timeSinceLastFrame;
            if (ene.timeElapsed > ene.cooldown) {
                ene.timeElapsed = 0.0;
                evilShot(ene.owner, Ogre::Math::UnitRandom()*70);
            }
        }
		return true;
	}
	return false;
}
bool TutorialApplication::keyPressed( const OIS::KeyEvent &arg ) {
	switch (arg.key)
    {
		case OIS::KC_W:
			mDirection.z = -mMove;
			break;
		case OIS::KC_S:
			mDirection.z = mMove;
			break;
		case OIS::KC_A:
			mDirection.x = -mMove;
			break;
		case OIS::KC_D:
			mDirection.x = mMove;
			break;
		case OIS::KC_E:
			mDirection.y = -mMove;
			break;
		case OIS::KC_Q:
			mDirection.y = mMove;
			break;
        default:
            break;
    }

	return BaseApplication::keyPressed(arg);
}
bool TutorialApplication::keyReleased( const OIS::KeyEvent &arg ) {
	switch (arg.key) {
		case OIS::KC_W:
			mDirection.z = 0;
			break;
		case OIS::KC_S:
			mDirection.z = 0;
			break;
		case OIS::KC_A:
			mDirection.x = 0;
			break;
		case OIS::KC_D:
			mDirection.x = 0;
			break;
		case OIS::KC_E:
			mDirection.y = 0;
			break;
		case OIS::KC_Q:
			mDirection.y = 0;
			break;
        case OIS::KC_P:
            PhysicsManager::reference()->set_debug_draw_enabled( !PhysicsManager::reference()->debug_draw_enabled() );
            break;
		default:
			break;
	}
	return BaseApplication::keyReleased(arg);
}
bool TutorialApplication::mouseMoved( const OIS::MouseEvent &arg ) {
	if (mTrayMgr->injectMouseMove(arg)) return true;
    camera->injectMouseMoved(arg);
    return true;
}
bool TutorialApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id ) {
	if (mTrayMgr->injectMouseDown(arg, id)) return true;
    mCameraMan->injectMouseDown(arg, id);

	if (id == OIS::MB_Left) {
		shoot();
	}
    else if (id == OIS::MB_Right) {
        summonEvilBall();
    }
    return true;
}
bool TutorialApplication::buttonPressed( const OIS::JoyStickEvent &arg, int button ) {
    switch (button) {
    case 0:
        shoot();
        break;
    case 2:
        PhysicsManager::reference()->set_debug_draw_enabled( !PhysicsManager::reference()->debug_draw_enabled());
        break;
    case 3:
        summonEvilBall();
        break;
    case 4:
        mDirection.y = mMove;
        break;
    case 5:
        mDirection.y = -mMove;
        break;
    case 6:
        shoot();
        break;
    default:
        break;
    }
    return true;
}
bool TutorialApplication::buttonReleased( const OIS::JoyStickEvent &arg, int button ) {
    switch (button) {
    case 4:
        mDirection.y = 0;
        break;
    case 5:
        mDirection.y = 0;
        break;
    default:
        break;
    }
    return true;
}
bool TutorialApplication::axisMoved( const OIS::JoyStickEvent &arg, int axis ) {
    if (axis == MOVE_FORWARD_AXIS) {
        if (Ogre::Math::Abs(arg.state.mAxes[axis].abs) > 256)
            mDirection.z = (mMove * arg.state.mAxes[axis].abs) / OIS::JoyStick::MAX_AXIS;
        else
            mDirection.z = 0;
    }
    if (axis == MOVE_SIDEWAYS_AXIS) {
        if (Ogre::Math::Abs(arg.state.mAxes[axis].abs) > 256)
            mDirection.x = (mMove * arg.state.mAxes[axis].abs) / OIS::JoyStick::MAX_AXIS;
        else
            mDirection.x = 0;
    }
    return true;
}
bool TutorialApplication::sliderMoved( const OIS::JoyStickEvent &arg, int index) {
    return true;
}

void TutorialApplication::createPlane(const std::string& name, const Ogre::Vector3& dir, double dist, double width, double height) {
	Ogre::Plane plane(dir, dist);
 
    Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, width, height, 5, 5, true, 1, 5, 5, dir.perpendicular());
 
    const std::string wat = name+ "Entity";
    Ogre::Entity* entGround = mSceneMgr->createEntity(wat, name);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
 
    entGround->setMaterialName("Examples/Rockwall");
    entGround->setCastShadows(false);

    ShipProject::GameObject* ground = new ShipProject::GameObject(entGround, 0);
    ground->SetupCollision(new btStaticPlaneShape(btVector3(dir.x,dir.y,dir.z), dist), CollisionGroup::WALLS,
        CollisionGroup::BALLS | CollisionGroup::PLAYER | CollisionGroup::PROJECTILES_BALLS | CollisionGroup::PROJECTILES_PLAYER);
    ground->body()->setRestitution(0.6);
    objects_.push_back( ground );
}
void TutorialApplication::createSphere(const std::string& strName, const float r, const int nRings, const int nSegments) {
	Ogre::MeshPtr pSphere = Ogre::MeshManager::getSingleton().createManual(Ogre::String(strName), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::SubMesh *pSphereVertex = pSphere->createSubMesh();
 
	pSphere->sharedVertexData = new Ogre::VertexData();
	Ogre::VertexData* vertexData = pSphere->sharedVertexData;
 
	// define the vertex format
	Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
 
	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	Ogre::HardwareVertexBufferSharedPtr vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
 
	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	Ogre::HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
 
	float fDeltaRingAngle = (Ogre::Math::PI / nRings);
	float fDeltaSegAngle = (2 * Ogre::Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;
 
	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings; ring++ ) {
		float r0 = r * sinf (ring * fDeltaRingAngle);
		float y0 = r * cosf (ring * fDeltaRingAngle);
 
        // Generate the group of segments for the current ring
        for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
            float z0 = r0 * cosf(seg * fDeltaSegAngle);
 
            // Add one vertex to the strip which makes up the sphere
            *pVertex++ = x0;
            *pVertex++ = y0;
            *pVertex++ = z0;
 
            Ogre::Vector3 vNormal = Ogre::Vector3(x0, y0, z0).normalisedCopy();
            *pVertex++ = vNormal.x;
            *pVertex++ = vNormal.y;
            *pVertex++ = vNormal.z;
 
            *pVertex++ = (float) seg / (float) nSegments;
            *pVertex++ = (float) ring / (float) nRings;
 
            if (ring != nRings) {
				// each vertex (except the last) has six indices pointing to it
                *pIndices++ = wVerticeIndex + nSegments + 1;
                *pIndices++ = wVerticeIndex;               
                *pIndices++ = wVerticeIndex + nSegments;
                *pIndices++ = wVerticeIndex + nSegments + 1;
                *pIndices++ = wVerticeIndex + 1;
                *pIndices++ = wVerticeIndex;
                wVerticeIndex ++;
            }
        }; // end for seg
    } // end for ring
 
	// Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;
 
    // the original code was missing this line:
    pSphere->_setBounds( Ogre::AxisAlignedBox( Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r) ), false );
    pSphere->_setBoundingSphereRadius(r);
    // this line makes clear the mesh is loaded (avoids memory leaks)
    pSphere->load();
}

void TutorialApplication::shoot() {
    //Ogre::InstancedEntity* ball = balls->createInstancedEntity("Ogre/Skin");
	Ogre::Entity* ball = mSceneMgr->createEntity("projectile");
	//ball->setCastShadows(true);
	ball->setMaterialName("Projectile/Blue");

	Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    Ogre::Quaternion orient = camera->actual_orientation();
    node->setPosition(player->entity()->getParentSceneNode()->getPosition() );
	node->setOrientation(orient);
	node->translate( orient * (Ogre::Vector3::UNIT_Z * -0.5) );
	node->attachObject(ball);

    ShipProject::GameObject* oBall = new ShipProject::GameObject(ball, 0.25);
    oBall->SetupCollision(new btSphereShape(PROJECTILE_RADIUS), CollisionGroup::PROJECTILES_PLAYER, CollisionGroup::BALLS | CollisionGroup::WALLS);
    objects_.push_back( oBall );

    // orientatation * (original model forward vector) = direction vector
    oBall->body()->setLinearVelocity( BtOgre::Convert::toBullet(node->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z * 100));
    oBall->body()->setRestitution(1.0);
    oBall->body()->activate(true);

    projectiles_[oBall->entity_name()] = Projectile(oBall, 5.0);
}
void TutorialApplication::evilShot(ShipProject::GameObject* enemy, double speed) {
    Ogre::Entity* ball = mSceneMgr->createEntity("projectile");
	ball->setMaterialName("Projectile/Red");

	Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	Ogre::Vector3 dir = player->entity()->getParentSceneNode()->getPosition() - enemy->entity()->getParentSceneNode()->getPosition();
	dir.normalise();
    Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(dir);
    node->setPosition(enemy->entity()->getParentSceneNode()->getPosition() );
	node->setOrientation(orient);
	node->translate( orient * (Ogre::Vector3::UNIT_Z * -0.5) );
	node->attachObject(ball);

    ShipProject::GameObject* oBall = new ShipProject::GameObject(ball, 0.25);
    oBall->SetupCollision(new btSphereShape(PROJECTILE_RADIUS), CollisionGroup::PROJECTILES_BALLS, CollisionGroup::PLAYER | CollisionGroup::WALLS);
    objects_.push_back( oBall );

    // orientatation * (original model forward vector) = direction vector
    oBall->body()->setLinearVelocity( BtOgre::Convert::toBullet(node->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z * speed));
    oBall->body()->setRestitution(1.0);
    oBall->body()->activate(true);

    projectiles_[oBall->entity_name()] = Projectile(oBall, 5.0);
}

void TutorialApplication::summonEvilBall() {
    Ogre::Entity* ball = mSceneMgr->createEntity("evilBall");
	ball->setMaterialName("Ogre/Skin");

	Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    node->setPosition(Ogre::Vector3(Ogre::Math::SymmetricRandom() * (AREA_RANGE/2),
                                    Ogre::Math::SymmetricRandom() * (AREA_RANGE/2),
                                    Ogre::Math::SymmetricRandom() * (AREA_RANGE/2)));
	node->attachObject(ball);

    ShipProject::GameObject* oBall = new ShipProject::GameObject(ball, 2);
    oBall->SetupCollision(new btSphereShape(BALL_RADIUS), CollisionGroup::BALLS, CollisionGroup::PLAYER | CollisionGroup::WALLS 
                                                   | CollisionGroup::BALLS | CollisionGroup::PROJECTILES_PLAYER);
    objects_.push_back( oBall );

    if (Ogre::Math::UnitRandom() < 0.5) {
        oBall->body()->setLinearVelocity( btVector3(Ogre::Math::SymmetricRandom(), 
                                                    Ogre::Math::SymmetricRandom(), 
                                                    Ogre::Math::SymmetricRandom()) * Ogre::Math::RangeRandom(0.0, 70.0) );
    }
    oBall->body()->setRestitution(1.0);
    oBall->body()->activate(true);
    
    enemies_[oBall->entity_name()] = Enemy(oBall, Ogre::Math::UnitRandom() * 5);
}

void TutorialApplication::handleCollisions(GameObject* obj1, GameObject* obj2, btManifoldPoint& pt) {
    if (obj1->collision_group() == CollisionGroup::PROJECTILES_PLAYER) {
        if (obj2->collision_group() == CollisionGroup::BALLS) {
            /*objects_.remove(obj1);
            objects_.remove(obj2);
            projectiles_.erase(obj1->entity_name());
            enemies_.erase(obj2->entity_name());
            delete obj1;
            delete obj2;*/
            if (std::find(projs_to_remove_.begin(), projs_to_remove_.end(), obj1->entity_name()) == projs_to_remove_.end() )
                projs_to_remove_.push_back(obj1->entity_name());
            if (std::find(enemies_to_remove_.begin(), enemies_to_remove_.end(), obj2->entity_name()) == enemies_to_remove_.end() )
                enemies_to_remove_.push_back(obj2->entity_name());
        }
        else if (obj2->collision_group() == CollisionGroup::WALLS) {
        }
    }
    else if (obj1->collision_group() == CollisionGroup::PROJECTILES_BALLS) {
        if (obj2->collision_group() == CollisionGroup::PLAYER) {
            /*objects_.remove(obj1);
            projectiles_.erase(obj1->entity_name());
            delete obj1;*/
            if (std::find(projs_to_remove_.begin(), projs_to_remove_.end(), obj1->entity_name()) == projs_to_remove_.end() )
                projs_to_remove_.push_back(obj1->entity_name());
        }
        else if (obj2->collision_group() == CollisionGroup::WALLS) {
        }
    }
    else if (obj1->collision_group() == CollisionGroup::BALLS) {
        if (obj2->collision_group() == CollisionGroup::PROJECTILES_PLAYER) {
            handleCollisions(obj2, obj1, pt);
        }
    }
    else if (obj1->collision_group() == CollisionGroup::PLAYER) {
        if (obj2->collision_group() == CollisionGroup::PROJECTILES_BALLS) {
            handleCollisions(obj2, obj1, pt);
        }
    }
    else if (obj1->collision_group() == CollisionGroup::WALLS) {
    }
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif
int main(int argc, char *argv[])
{
    // Create application object
    TutorialApplication app;

    try {
        app.go();
    } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        std::cerr << "An exception has occured: " <<
            e.getFullDescription().c_str() << std::endl;
#endif
    }

    return 0;
}

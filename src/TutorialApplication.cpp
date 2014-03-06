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

using ShipProject::PhysicsManager;

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void) {
	mRotate = 0.13;
	mMove = 250;
	mDirection = Ogre::Vector3::ZERO;
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void) {
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void) {
	PhysicsManager::reference()->Initialize(btVector3(0,-10,0), mSceneMgr);

    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    //mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
 
    Ogre::Entity* playerEnt = mSceneMgr->createEntity("Ninja", "ninja.mesh");
    playerEnt->setCastShadows(true);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(playerEnt);
    double ph = playerEnt->getBoundingBox().getSize().y;
    BtOgre::StaticMeshToShapeConverter converter (playerEnt);
    btTransform initialOffset(btQuaternion::getIdentity(), btVector3(0,-ph/2,0));
    player = new ShipProject::GameObject(playerEnt, 80, converter.createCapsule(), initialOffset);
    player->body()->setAngularFactor(btVector3(0.0, 0.0, 0.0));
    objects_.push_back( player );

    mCamera->setPosition(Ogre::Vector3(0,ph*0.85,500));
    mCamera->lookAt(Ogre::Vector3(0,ph*0.85,0));
	playerEnt->getParentSceneNode()->createChildSceneNode("camNode")->attachObject(mCamera);
 
	this->createPlane("ground", Ogre::Vector3::UNIT_Y, 0);
	this->createPlane("ceiling", -Ogre::Vector3::UNIT_Y, -500);
	this->createPlane("leftwall", Ogre::Vector3::UNIT_X, -500);
	this->createPlane("rightwall", -Ogre::Vector3::UNIT_X, -500);
	this->createPlane("backwall", Ogre::Vector3::UNIT_Z, -500);
	this->createPlane("frontwall", -Ogre::Vector3::UNIT_Z, -500);

	// LIGHTS
	Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setPosition(Ogre::Vector3(0, 150, 250));
 
    pointLight->setDiffuseColour(1.0, 0.0, 0.0);
    pointLight->setSpecularColour(1.0, 0.0, 0.0);
 
    Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(.25, .25, 0));
    directionalLight->setSpecularColour(Ogre::ColourValue(.25, .25, 0));
 
    directionalLight->setDirection(Ogre::Vector3( 0, -1, 1 )); 
 
    Ogre::Light* spotLight = mSceneMgr->createLight("spotLight");
    spotLight->setType(Ogre::Light::LT_SPOTLIGHT);
    spotLight->setDiffuseColour(0, 0, 1.0);
    spotLight->setSpecularColour(0, 0, 1.0);
 
    spotLight->setDirection(-1, -1, 0);
    spotLight->setPosition(Ogre::Vector3(300, 300, 0));
    spotLight->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));

	// GOTTA CHECK IF WE SUPPORT THE INSTANCING TECHNIQUE (involves hardware and software)
	createSphere("mySphereMesh", 15);
	//balls = mSceneMgr->createInstanceManager("balls", "mySphereMesh", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME, 
	//	Ogre::InstanceManager::InstancingTechnique::HWInstancingVTF, 100, Ogre::IM_USEALL);
}
void TutorialApplication::destroyScene() {
    for (auto obj : objects_) {
        delete obj;
    }
    objects_.clear();

	delete PhysicsManager::reference();
}

void TutorialApplication::createCamera(void)
{
    // create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");
    
    // set the near clip distance
    mCamera->setNearClipDistance(5);
 
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
        player->Move( player->entity()->getParentSceneNode()->getOrientation() * mDirection );
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
	/*player->getParentSceneNode()->yaw(Ogre::Degree(-mRotate * arg.state.X.rel), Ogre::Node::TS_WORLD);
    player->getParentSceneNode()->pitch(Ogre::Degree(-mRotate * arg.state.Y.rel), Ogre::Node::TS_LOCAL);*/
    player->Rotate(-mRotate * arg.state.X.rel, -mRotate * arg.state.Y.rel, 0);
    return true;
}
bool TutorialApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id ) {
	if (mTrayMgr->injectMouseDown(arg, id)) return true;
    mCameraMan->injectMouseDown(arg, id);

	if (id == OIS::MB_Left) {
		//Ogre::InstancedEntity* ball = balls->createInstancedEntity("Ogre/Skin");
		Ogre::Entity* ball = mSceneMgr->createEntity("mySphereMesh");
		//ball->setCastShadows(true);
		ball->setMaterialName("Ogre/Skin");

		Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		node->setPosition(player->entity()->getParentSceneNode()->getPosition() + Ogre::Vector3(0.0, player->entity()->getBoundingBox().getSize().y*0.8, 0.0) );
		node->setOrientation(player->entity()->getParentSceneNode()->getOrientation() );
		node->translate( node->getOrientation() * (Ogre::Vector3::UNIT_Z * -50) );
		node->attachObject(ball);

        ShipProject::GameObject* oBall = new ShipProject::GameObject(ball, 1, new btSphereShape(15));
        objects_.push_back( oBall );

        // orientatation * (original model forward vector) = direction vector
        oBall->body()->setLinearVelocity( BtOgre::Convert::toBullet(node->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z * 300) );
        oBall->body()->setRestitution(1.0);
	}
    return true;
}

void TutorialApplication::createPlane(const std::string& name, const Ogre::Vector3& dir, double dist) {
	Ogre::Plane plane(dir, dist);
 
    Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, 1000, 1000, 20, 20, true, 1, 5, 5, dir.perpendicular());
 
    Ogre::Entity* entGround = mSceneMgr->createEntity(name+"Entity", name);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
 
    entGround->setMaterialName("Examples/Rockwall");
    entGround->setCastShadows(false);

    ShipProject::GameObject* ground = new ShipProject::GameObject(entGround, 0, new btStaticPlaneShape(btVector3(dir.x,dir.y,dir.z), dist));
    ground->body()->setRestitution(0.6);
    objects_.push_back( ground );
}
void TutorialApplication::createSphere(const std::string& strName, const float r, const int nRings, const int nSegments) {
	Ogre::MeshPtr pSphere = Ogre::MeshManager::getSingleton().createManual(strName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
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
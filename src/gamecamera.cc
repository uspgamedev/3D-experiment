#include <gamecamera.h>
#include <gameobject.h>

#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OISMouse.h>

namespace ShipProject {

using Ogre::Vector3;
using Ogre::Quaternion;

GameCamera::GameCamera(Ogre::SceneManager* sceneMgr, const std::string& camName) {
    parent_ = nullptr;
    camera_ = sceneMgr->createCamera(camName);
    camera_->setNearClipDistance(1);
    dist_ = 0;
    offset_ = Vector3::ZERO;
    max_dist_ = 7.5;
}

GameCamera::~GameCamera() {
    //TODO: we should probably delete the camera here
}

void GameCamera::AttachTo(GameObject* object) {
    if (parent_ != nullptr) {
        Ogre::SceneNode* oldCamNode = camera_->getParentSceneNode();
        parent_->entity()->getParentSceneNode()->removeChild( oldCamNode );
        camera_->detachFromParent();
        delete oldCamNode;
    }
    parent_ = object;
    parent_->entity()->getParentSceneNode()->createChildSceneNode()->attachObject(camera_);

    setupTransform();
}

void GameCamera::SetParameters(const Vector3& parent_origin_offset, double max_dist) {
    offset_ = parent_origin_offset;
    if (max_dist_ > 0)
        max_dist_ = max_dist;
    setupTransform();
}

Quaternion GameCamera::orientation() {
    if (parent_ != nullptr)
        return camera_->getParentSceneNode()->getOrientation();
    return Quaternion::IDENTITY;
}

void GameCamera::injectMouseMoved( const OIS::MouseEvent &arg ) {
    Ogre::SceneNode* node = camera_->getParentSceneNode();
    node->yaw(Ogre::Degree(-0.13 * arg.state.X.rel), Ogre::Node::TS_WORLD);
    node->pitch(Ogre::Degree(-0.13 * arg.state.Y.rel), Ogre::Node::TS_LOCAL);

    dist_ += ((arg.state.Z.rel > 0) ? -1 : 1) * 0.225;
    if (dist_ <= 0) dist_ = 0;
    if (dist_ > max_dist_) dist_ = max_dist_;
    Vector3 pos = parent_->entity()->getParentSceneNode()->getOrientation() * Vector3::UNIT_Z * dist_;
    camera_->setPosition( pos );
}

void GameCamera::setupTransform() {
    if (parent_ == nullptr) return;
    Ogre::SceneNode* node = camera_->getParentSceneNode();

    node->setPosition( offset_ );
    node->setOrientation( Quaternion::IDENTITY );
    camera_->setPosition( parent_->entity()->getParentSceneNode()->getOrientation() * Vector3::UNIT_Z * dist_ );
}

}

#include <gameobject.h>

#include <btBulletDynamicsCommon.h>
#include <physicsmanager.h>

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include <BtOgreExtras.h>
#include <BtOgrePG.h>


namespace ShipProject {

GameObject::GameObject(Ogre::Entity* entity, double mass, btCollisionShape* shape) : entity_(entity), mass_(mass), shape_(shape) {
    //GameObjectMotionState* motionState = new GameObjectMotionState(entity);
    Ogre::Quaternion orient = entity->getParentSceneNode()->getOrientation();
    Ogre::Vector3 pos = entity->getParentSceneNode()->getPosition();
    btTransform t = btTransform( BtOgre::Convert::toBullet(orient), BtOgre::Convert::toBullet(pos));
    BtOgre::RigidBodyState* motionState = new BtOgre::RigidBodyState(entity->getParentSceneNode(), t);

    btVector3 inertia(0,0,0);
    if (mass_ > 0.0)
        shape_->calculateLocalInertia(mass_,inertia);
    btRigidBody::btRigidBodyConstructionInfo  bodyInfo(mass_,motionState,shape_,inertia);
    body_ = new btRigidBody(bodyInfo);
    body_->setLinearFactor(btVector3(1,1,1));
	body_->setActivationState(DISABLE_DEACTIVATION);

	PhysicsManager::reference()->AddBody(body_);
}
GameObject::GameObject(Ogre::Entity* entity, double mass, btCollisionShape* shape, const btTransform& offset) 
    : entity_(entity), mass_(mass), shape_(shape) 
{
    //GameObjectMotionState* motionState = new GameObjectMotionState(entity);
    Ogre::Quaternion orient = entity->getParentSceneNode()->getOrientation();
    Ogre::Vector3 pos = entity->getParentSceneNode()->getPosition();
    btTransform t = btTransform( BtOgre::Convert::toBullet(orient), BtOgre::Convert::toBullet(pos));
    BtOgre::RigidBodyState* motionState = new BtOgre::RigidBodyState(entity->getParentSceneNode(), t, offset);

    btVector3 inertia(0,0,0);
    if (mass_ > 0.0)
        shape_->calculateLocalInertia(mass_,inertia);
    btRigidBody::btRigidBodyConstructionInfo  bodyInfo(mass_,motionState,shape_,inertia);
    body_ = new btRigidBody(bodyInfo);
    body_->setLinearFactor(btVector3(1,1,1));
	body_->setActivationState(DISABLE_DEACTIVATION);

	PhysicsManager::reference()->AddBody(body_);
}

GameObject::~GameObject() {
    PhysicsManager::reference()->RemoveBody(body_);
    delete body_->getMotionState();
    delete body_;
    delete shape_;
}

void GameObject::Translate(const Ogre::Vector3& move) {
    body_->translate(BtOgre::Convert::toBullet(move));
}
void GameObject::Move(const Ogre::Vector3& delta) {
    body_->activate();
    body_->applyCentralImpulse(BtOgre::Convert::toBullet(delta));
}
void GameObject::Rotate(double yaw, double pitch, double roll) {
    //body_->activate();    
    //body_->applyTorque(btVector3(yaw, pitch, roll));

    btTransform t = body_->getCenterOfMassTransform();
    Ogre::Quaternion rot = BtOgre::Convert::toOgre(t.getRotation());

    Ogre::Matrix3 ypr = Ogre::Matrix3::IDENTITY;
    ypr.FromEulerAnglesXYZ(Ogre::Radian(yaw), Ogre::Radian(pitch), Ogre::Radian(roll));
    t.setRotation( BtOgre::Convert::toBullet(rot * ypr) );
    body_->setCenterOfMassTransform(t);
}

}

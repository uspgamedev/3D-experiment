
#ifndef GAMEOBJECT_H_
#define GAMEOBJECT_H_

#include <btBulletDynamicsCommon.h>

namespace Ogre {
class Entity;
class Vector3;
}

class btCollisionShape;
class btRigidBody;

namespace ShipProject {

class GameObject {
public:
    GameObject(Ogre::Entity* entity, double mass, btCollisionShape* shape);
    GameObject(Ogre::Entity* entity, double mass, btCollisionShape* shape, const btTransform& offset);
    ~GameObject();

    Ogre::Entity* entity() { return entity_; }
	btRigidBody* body() { return body_; }

    void Translate(const Ogre::Vector3& move);
    void Move(const Ogre::Vector3& delta);
    void Rotate(double yaw, double pitch, double roll);

protected:
	Ogre::Entity* entity_;

	double mass_;
	btCollisionShape* shape_;
	btRigidBody* body_;
};

}
#endif

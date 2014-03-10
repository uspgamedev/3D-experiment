
#ifndef GAMEOBJECT_H_
#define GAMEOBJECT_H_

#include <btBulletDynamicsCommon.h>
#include <string>

namespace Ogre {
class Entity;
class Vector3;
}

class btCollisionShape;
class btRigidBody;

namespace ShipProject {

#define BIT(x) (1<<(x))
enum CollisionGroup {
    WALLS = BIT(6),
    PLAYER = BIT(7),
    BALLS = BIT(8),
    PROJECTILES_PLAYER = BIT(9),
    PROJECTILES_BALLS = BIT(10)
};

class GameObject {
public:
    GameObject(Ogre::Entity* entity, double mass);
    ~GameObject();

    Ogre::Entity* entity() const { return entity_; }
	btRigidBody* body() const { return body_; }
    std::string entity_name();

    void SetupCollision(btCollisionShape* shape, short collisionGroup, short collidesWith, const btTransform& offset = btTransform::getIdentity());
    short collision_group() const { return collision_group_; }
    short collides_with() const { return collides_with_; }

    void Translate(const Ogre::Vector3& move);
    void Move(const Ogre::Vector3& delta);
    void Rotate(double yaw, double pitch, double roll);

protected:
	Ogre::Entity* entity_;

	double mass_;
	btCollisionShape* shape_;
	btRigidBody* body_;
    short collision_group_;
    short collides_with_;
};

}
#endif

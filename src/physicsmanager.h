#ifndef PHYSICSMANAGER_H_
#define PHYSICSMANAGER_H_


class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btVector3;

namespace BtOgre {
class DebugDrawer;
}
namespace Ogre {
class SceneManager;
}

namespace ShipProject {

class PhysicsManager {
public:
	static PhysicsManager* reference() { return reference_ ? reference_ : reference_ = new PhysicsManager; }

	~PhysicsManager();

    void Initialize(const btVector3& grav, Ogre::SceneManager* sceneMgr);
    void Update(double dt);

    void AddBody(btRigidBody* body);
    void RemoveBody(btRigidBody* body);

    void set_debug_draw_enabled(bool enable);
    bool debug_draw_enabled();

private:
	static PhysicsManager* reference_;

	PhysicsManager();

    btBroadphaseInterface* broadphase_;
    btDefaultCollisionConfiguration* config_;
    btCollisionDispatcher* dispatcher_;
    btSequentialImpulseConstraintSolver* solver_;
    btDiscreteDynamicsWorld* world_;

    BtOgre::DebugDrawer* debug_drawer_;
};

}
#endif

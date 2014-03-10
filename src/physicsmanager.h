#ifndef PHYSICSMANAGER_H_
#define PHYSICSMANAGER_H_

#include <functional>
#include <btBulletDynamicsCommon.h>

namespace BtOgre {
class DebugDrawer;
}
namespace Ogre {
class SceneManager;
}

namespace ShipProject {
class GameObject;

typedef std::function<void (GameObject*,GameObject*,btManifoldPoint&)> CollisionLogic;

class PhysicsManager {
public:
	static PhysicsManager* reference() { return reference_ ? reference_ : reference_ = new PhysicsManager; }

	~PhysicsManager();

    void Initialize(const btVector3& grav, Ogre::SceneManager* sceneMgr);
    void Update(double dt);

    void AddBody(GameObject* obj);
    void RemoveBody(GameObject* obj);

    void set_debug_draw_enabled(bool enable);
    bool debug_draw_enabled();

    void set_collision_logic(CollisionLogic& logic) { col_logic_ = logic; }
    CollisionLogic& collision_logic() { return col_logic_; }

private:
	static PhysicsManager* reference_;

	PhysicsManager();

    btBroadphaseInterface* broadphase_;
    btDefaultCollisionConfiguration* config_;
    btCollisionDispatcher* dispatcher_;
    btSequentialImpulseConstraintSolver* solver_;
    btDiscreteDynamicsWorld* world_;

    BtOgre::DebugDrawer* debug_drawer_;

    CollisionLogic col_logic_;
};

}
#endif

/*******************************************************************/
/**   MAC 420 - Introdução à Computação Gráfica                   **/
/**   IME-USP - Primeiro Semestre de 2012                         **/
/**   BCC2009 - Marcel P. Jackowski                               **/
/**                                                               **/
/**   Segundo Exercício-Programa                                  **/
/**   Arquivo: physicsmanager.cc                                  **/
/**                                                               **/
/**   Fernando Omar Aluani             #USP: 6797226              **/
/**                                                               **/
/**   Entregado em 03/07/2012                                     **/
/*******************************************************************/
#include "physicsmanager.h"
#include <btBulletDynamicsCommon.h>
#include <BtOgreExtras.h>
#include <OgreSceneManager.h>
#include <gameobject.h>

namespace ShipProject {

PhysicsManager* PhysicsManager::reference_ = nullptr;

PhysicsManager::PhysicsManager() {
}

PhysicsManager::~PhysicsManager() {
	delete broadphase_;
    delete config_;
    delete dispatcher_;
    delete solver_;
    delete debug_drawer_;
    delete world_;
}

void tickCallback(btDynamicsWorld *world, btScalar timeStep);

void PhysicsManager::Initialize(const btVector3& grav, Ogre::SceneManager* sceneMgr) {
	// Broadphase is the initial collision detecting: checks for colliding pairs given their bounding boxes
	broadphase_ = new btDbvtBroadphase();
 
    // Set up the collision configuration and dispatcher
    config_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(config_);
 
    // The actual physics solver
    solver_ = new btSequentialImpulseConstraintSolver;
 
    // The world.
    world_ = new btDiscreteDynamicsWorld(dispatcher_,broadphase_,solver_,config_);
    world_->setGravity(grav);

	btContactSolverInfo& info = world_->getSolverInfo();
	info.m_splitImpulse = 1; //enable split impulse feature
    
    debug_drawer_ = new BtOgre::DebugDrawer(sceneMgr->getRootSceneNode(), world_);
    debug_drawer_->setDebugMode(false);
    world_->setDebugDrawer(debug_drawer_);

    world_->setInternalTickCallback( &tickCallback );
}

void PhysicsManager::Update(double dt) {
    // stepSimulation( dt, maxSubSteps, fixedDtSubStep=1/60)
    // dt < maxSubSteps * fixedDtSubStep
    world_->stepSimulation(dt, 10);
    debug_drawer_->step();
}

void PhysicsManager::AddBody(GameObject* obj) {
    world_->addRigidBody(obj->body(), obj->collision_group(), obj->collides_with());
}
void PhysicsManager::RemoveBody(GameObject* obj) {
    world_->removeRigidBody(obj->body());
}

void PhysicsManager::set_debug_draw_enabled(bool enable) { 
    debug_drawer_->setDebugMode(enable);
}
bool PhysicsManager::debug_draw_enabled() { 
    return debug_drawer_->getDebugMode() != 0; 
}

void tickCallback(btDynamicsWorld *world, btScalar timeStep) {
    int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
        GameObject* obA = static_cast<GameObject*>(contactManifold->getBody0()->getUserPointer());
		GameObject* obB = static_cast<GameObject*>(contactManifold->getBody1()->getUserPointer());

		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
            //std::cout << obA->entity_name() << " IS NEAR (" << pt.getDistance() << ") " << obB->entity_name() << std::endl;
			if (pt.getDistance() <= 0)
			{
                CollisionLogic& logic = PhysicsManager::reference()->collision_logic();
                if (logic) {
                    logic(obA, obB, pt);
                }
                //std::cout << obA->entity_name() << "(" << ptA.x() << ", " << ptA.y() << ", "<<ptA.z() << ") collided with " << 
                //    obB->entity_name() << "(" << ptB.x() << ", " << ptB.y() << ", "<<ptB.z() << ")" << std::endl;
			}
		}
	}
}

}

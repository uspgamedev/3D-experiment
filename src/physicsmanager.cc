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
}

void PhysicsManager::Update(double dt) {
    world_->stepSimulation(dt, 10);
    debug_drawer_->step();
}

void PhysicsManager::AddBody(btRigidBody* body) {
    world_->addRigidBody(body);
}
void PhysicsManager::RemoveBody(btRigidBody* body) {
    world_->removeRigidBody(body);
}

void PhysicsManager::set_debug_draw_enabled(bool enable) { 
    debug_drawer_->setDebugMode(enable);
}
bool PhysicsManager::debug_draw_enabled() { 
    return debug_drawer_->getDebugMode() != 0; 
}
}

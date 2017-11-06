#include "my_physics_sim.h"

MyPhysicsSim::MyPhysicsSim()
{
	collision_config = new btDefaultCollisionConfiguration();
	dispatcher = new	btCollisionDispatcher(collision_config);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	dynamics_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_config);
	
	dynamics_world->setGravity(btVector3(0, 0, 0));
	btContactSolverInfo& slvInfo = dynamics_world->getSolverInfo();
	slvInfo.m_erp = 0.8;

	ground_shape = NULL;
	ground_body = NULL;
}

MyPhysicsSim::~MyPhysicsSim()
{
	for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		dynamics_world->removeCollisionObject(obj);
		delete obj;
	}

	// delete constraints (joints)
	for (int i = dynamics_world->getNumConstraints() - 1; i >= 0; i--) {
		btTypedConstraint* constraint = dynamics_world->getConstraint(i);
		dynamics_world->removeConstraint(constraint);
		delete constraint;
	}

	delete dynamics_world;
	delete solver;
	delete broadphase;
	delete dispatcher;
	delete collision_config;
	ground_shape = NULL;
	ground_body = NULL;

}

btRigidBody * MyPhysicsSim::addRB(btScalar mass, const btTransform & initTransform, btCollisionShape * shape, short group, short mask)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);


	btDefaultMotionState* myMotionState = new btDefaultMotionState(initTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);

	body->setContactProcessingThreshold(BT_LARGE_FLOAT);

	dynamics_world->addRigidBody(body, group, mask);

	return body;
}

void MyPhysicsSim::addConstraint(btTypedConstraint * constraint, bool disableCollisionsBetweenLinkedBodies)
{
	dynamics_world->addConstraint(constraint, disableCollisionsBetweenLinkedBodies);
}

int MyPhysicsSim::stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	return dynamics_world->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

void MyPhysicsSim::setGround(float height)
{
	btTransform ts;

	if (ground_body != NULL)
	{
		delete ground_shape;
		delete ground_body->getMotionState();
		dynamics_world->removeCollisionObject(ground_body);
		delete ground_body;
		ground_body = NULL;
	}

	ground_shape = new btBoxShape(btVector3(10, 1.0, 10));
	ts.setIdentity();
	ts.setOrigin(btVector3(0.0, height - 1.0, 0.0));
	ground_body = addRB(0, ts, ground_shape, COL_FLOOR, COL_SKELTON);
	ground_body->setRestitution(0.0);
	ground_body->setFriction(0.0);
}

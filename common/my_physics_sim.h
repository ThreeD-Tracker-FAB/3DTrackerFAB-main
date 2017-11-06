#pragma once

#include "btBulletDynamicsCommon.h"

enum collisiontypes {
	COL_NOTHING = 0,
	COL_FLOOR = ((unsigned long long)1) << 0,
	COL_SKELTON = ((unsigned long long)1) << 1,
};

class MyPhysicsSim
{
public:

	MyPhysicsSim();
	~MyPhysicsSim();

	btRigidBody* addRB(btScalar mass, const btTransform& initTransform, btCollisionShape* shape,
		short group, short mask);

	void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies = false);

	int	stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));

	void setGround(float height);

private:

	btDiscreteDynamicsWorld* dynamics_world;
	btBroadphaseInterface* broadphase;
	btCollisionDispatcher* dispatcher;
	btConstraintSolver* solver;
	btDefaultCollisionConfiguration* collision_config;

	// ground
	btCollisionShape* ground_shape;
	btRigidBody* ground_body;
};


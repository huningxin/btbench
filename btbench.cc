#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#define DEBUG 0

#define WARMUP 2
#define FRAMES 64

#define BOX_COUNT 1000
#define ITERATIONS 100000

typedef struct {
  float mean;
  float pc_5th;
  float pc_95th;
} result_t;

using namespace std;

// Simple nearest-rank %ile (on sorted array). We should have enough samples to make this reasonable.
float percentile(clock_t times[FRAMES], float pc) {
  int rank = (int)((pc * FRAMES) / 100);
  return times[rank];
}

int _cmp(const void *a, const void *b) {
	return *(clock_t*)a - *(clock_t*)b;
}

result_t measure(clock_t times[FRAMES]) {
  float values[FRAMES];
  result_t r;

	float total = 0;
	for (int i = 0; i < FRAMES; ++i) {
		values[i] = (float)times[i] / CLOCKS_PER_SEC * 1000;
	 	total += values[i];
	}
  r.mean = total / FRAMES;

	qsort(times, FRAMES, sizeof(clock_t), _cmp);
  r.pc_5th = percentile(times, 5) / CLOCKS_PER_SEC * 1000;
  r.pc_95th = percentile(times, 95) / CLOCKS_PER_SEC * 1000;
  return r;
}

result_t bench_bullet() {
	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0,-10,0));

	///-----initialization_end-----

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

	std::vector<btRigidBody*> bodies;

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-56,0));

	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}


	{
		//create a dynamic rigidbody

		btCollisionShape* boxShape = new btBoxShape(btVector3(1,1,1));

		for (int i = 0; i < BOX_COUNT; ++i) {
			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar	mass(1.f);

			btVector3 localInertia(0,0,0);
			boxShape->calculateLocalInertia(mass,localInertia);

			startTransform.setOrigin(btVector3(2,10,0));
			
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,boxShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			dynamicsWorld->addRigidBody(body);

			bodies.push_back(body);
		}
	}

	//dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;


	printf("warming...\n");
	for (int i = 0; i < WARMUP; ++i) {
		///step the simulation
		dynamicsWorld->stepSimulation(btScalar(1./60.));
  	}

  	printf("benching...\n");
	clock_t times[FRAMES]; 
	for (int i = 0; i < FRAMES; ++i) {
		clock_t start = clock();
		dynamicsWorld->stepSimulation(btScalar(1./60.));
		clock_t end = clock();
		times[i] = end - start;
	}

  return measure(times);
}

result_t bench_btSequentialImpulseConstraintSolver() {
	btSolverBody body_template;
	btSolverConstraint constraint_template;

	body_template.m_worldTransform.setIdentity();
	body_template.m_deltaLinearVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_deltaAngularVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_angularFactor.setValue(1.01f,1.01f,1.01f);
	body_template.m_linearFactor.setValue(1.01f,1.01f,1.01f);
	body_template.m_invMass.setValue(1.01f,1.01f,1.01f);
	body_template.m_pushVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_turnVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_linearVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_angularVelocity.setValue(1.01f,1.01f,1.01f);
	body_template.m_externalForceImpulse.setValue(1.01f,1.01f,1.01f);
	body_template.m_externalTorqueImpulse.setValue(1.01f,1.01f,1.01f);
	body_template.m_originalBody = 0;

	constraint_template.m_relpos1CrossNormal.setValue(1.01f,1.01f,1.01f);
	constraint_template.m_relpos2CrossNormal.setValue(1.01f,1.01f,1.01f);
	constraint_template.m_angularComponentA.setValue(1.01f,1.01f,1.01f);
	constraint_template.m_angularComponentB.setValue(1.01f,1.01f,1.01f);
	
	constraint_template.m_appliedPushImpulse = 1.01f;
	constraint_template.m_appliedImpulse = 1.01f;
	
	constraint_template.m_friction = 1.01f;
	constraint_template.m_jacDiagABInv = 1.01f;
	constraint_template.m_rhs = 1.01f;
	constraint_template.m_cfm = 1.01f;
	constraint_template.m_lowerLimit = 0.01f;
	constraint_template.m_upperLimit = 1000.01f;
	constraint_template.m_rhsPenetration = 1.01f;

	btSequentialImpulseConstraintSolver solver;
	btSingleConstraintRowSolver signle_constraint_solver = solver.getActiveConstraintRowSolverGeneric();

	printf("warming...\n");
	for (int i = 0; i < WARMUP; ++i) {
		///step the simulation
		btSolverBody body = body_template;
		btSolverConstraint constraint = constraint_template;
		signle_constraint_solver(body, body, constraint);
  	}

  	float checksum = 0.0f;

  	printf("benching...\n");
	clock_t times[FRAMES]; 
	for (int i = 0; i < FRAMES; ++i) {
		int f = 0;
		clock_t start = clock();
		for (int j = 0; j < ITERATIONS; ++j) {
			btSolverBody body = body_template;
			btSolverConstraint constraint = constraint_template;
			checksum += signle_constraint_solver(body, body, constraint);
	    }
		clock_t end = clock();
		times[i] = end - start;
	}

  printf("Checksum: %f\n", checksum);

  return measure(times);
}


int main(int argc, char** argv) {
  result_t result;

  result = bench_bullet();
  printf("Simulation benchmark complete.\n  ms/frame: %f 5th %%ile: %f 95th %%ile: %f\n", result.mean, result.pc_5th, result.pc_95th);

  result = bench_btSequentialImpulseConstraintSolver();
  printf("btSequentialImpulseConstraintSolver benchmark complete.\n  ms/frame: %f 5th %%ile: %f 95th %%ile: %f\n", result.mean, result.pc_5th, result.pc_95th);
}

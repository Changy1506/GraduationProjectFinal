#include "TimeStepController.h"
#include "Simulation/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include <iostream>
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "Utils/Timing.h"

using namespace PBD;
using namespace std;
using namespace GenParam;

// int TimeStepController::SOLVER_ITERATIONS = -1;
// int TimeStepController::SOLVER_ITERATIONS_V = -1;
int TimeStepController::MAX_ITERATIONS = -1;
int TimeStepController::MAX_ITERATIONS_V = -1;
int TimeStepController::VELOCITY_UPDATE_METHOD = -1;
int TimeStepController::ENUM_VUPDATE_FIRST_ORDER = -1;
int TimeStepController::ENUM_VUPDATE_SECOND_ORDER = -1;
bool control = false;
float Treshold = 0.2;

struct jj
{
	int l, r;
	float minx, maxx, miny, maxy, minz, maxz;
}tree[60000];
int map[60000];
bool overlap2[2000][2000];

bool overlap(jj a, jj b)
{
	if (a.minx - b.maxx > Treshold || b.minx - a.maxx > Treshold)return false;
	if (a.miny - b.maxy > Treshold || b.miny - a.maxy > Treshold)return false;
	if (a.minz - b.maxz > Treshold || b.minz - a.maxz > Treshold)return false;
	//printf("OK\n");
	return true;
}

void build_tree(int pos, int l,int r, ParticleData &pd)
{
//	printf("%d %d\n", l, r);
	int mid = (l + r) / 2;
	tree[pos].l = l;
	tree[pos].r = r;
	//printf("%d %d %d\n", pos, tree[pos].l, tree[pos].r);
	tree[pos].minx = pd.getPosition(l)(0, 0);
	tree[pos].maxx = pd.getPosition(l)(0, 0);
	tree[pos].miny = pd.getPosition(l)(1, 0);
	tree[pos].maxy = pd.getPosition(l)(1, 0);
	tree[pos].miny = pd.getPosition(l)(2, 0);
	tree[pos].maxy = pd.getPosition(l)(2, 0);

	if (l == r) { map[l] = pos; return; }
	build_tree(pos + pos, l , mid, pd);
	build_tree(pos + pos + 1, mid + 1, r, pd);

	tree[pos].minx = min(tree[pos].minx, tree[pos + pos].minx);
	tree[pos].minx = min(tree[pos].minx, tree[pos + pos+1].minx);
	
	tree[pos].maxx = max(tree[pos].maxx, tree[pos + pos].maxx);
	tree[pos].maxx = max(tree[pos].maxx, tree[pos + pos + 1].maxx);

	tree[pos].miny = min(tree[pos].miny, tree[pos + pos].miny);
	tree[pos].miny = min(tree[pos].miny, tree[pos + pos + 1].miny);

	tree[pos].maxy = max(tree[pos].maxy, tree[pos + pos].maxy);
	tree[pos].maxy = max(tree[pos].maxy, tree[pos + pos + 1].maxy);

	tree[pos].minz = min(tree[pos].minz, tree[pos + pos].minz);
	tree[pos].minz = min(tree[pos].minz, tree[pos + pos + 1].minz);

	tree[pos].maxz = max(tree[pos].maxz, tree[pos + pos].maxz);
	tree[pos].maxz = max(tree[pos].maxz, tree[pos + pos + 1].maxz);
}
void tree_ask(int i, int j)
{
	if (!overlap(tree[i], tree[j]))return;
	if(tree[j].l == tree[j].r)
	{
		//int c = tree[j].r;
		if (i != j)
		{
		//	printf("yes!\n");
		}
	    overlap2[tree[i].l][tree[j].l] = true;
		//printf("%d %d\n", tree[i].l, tree[j].l);
		return;
	}
	tree_ask(i,j+j);
	tree_ask(i,j+j+1);
}
void update(int l,int r)
{
	//int i, j;
	for (int i = l; i <= r; i++)
		for (int j = l; j <= r; j++)
			overlap2[i][j] = false;
	for (int i = l; i < r; i++)
	{
		tree_ask(map[i], 1);
		//printf("%d ",map[i]);
	}
	//printf("\n");
}
void pre_solve_edgeedge(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();
	build_tree(1, 0, int(pd.size())-1, pd);
	update(0,int(pd.size()) - 1);
}
TimeStepController::TimeStepController() 
{
	m_velocityUpdateMethod = 0;
	m_iterations = 0;
	m_iterationsV = 0;
	m_maxIterations = 5;
	m_maxIterationsV = 5;
	m_collisionDetection = NULL;	
}

TimeStepController::~TimeStepController(void)
{
}

void TimeStepController::initParameters()
{
	TimeStep::initParameters();

// 	SOLVER_ITERATIONS = createNumericParameter("iterations", "Iterations", &m_iterations);
// 	setGroup(SOLVER_ITERATIONS, "PBD");
// 	setDescription(SOLVER_ITERATIONS, "Iterations required by the solver.");
// 	getParameter(SOLVER_ITERATIONS)->setReadOnly(true);

	MAX_ITERATIONS = createNumericParameter("maxIterations", "Max. iterations", &m_maxIterations);
	setGroup(MAX_ITERATIONS, "PBD");
	setDescription(MAX_ITERATIONS, "Maximal number of iterations of the solver.");
	static_cast<NumericParameter<unsigned int>*>(getParameter(MAX_ITERATIONS))->setMinValue(1);

// 	SOLVER_ITERATIONS_V = createNumericParameter("iterationsV", "Velocity iterations", &m_iterationsV);
// 	setGroup(SOLVER_ITERATIONS_V, "PBD");
// 	setDescription(SOLVER_ITERATIONS_V, "Iterations required by the velocity solver.");
// 	getParameter(SOLVER_ITERATIONS_V)->setReadOnly(true);

	MAX_ITERATIONS_V = createNumericParameter("maxIterationsV", "Max. velocity iterations", &m_maxIterationsV);
	setGroup(MAX_ITERATIONS_V, "PBD");
	setDescription(MAX_ITERATIONS_V, "Maximal number of iterations of the velocity solver.");
	static_cast<NumericParameter<unsigned int>*>(getParameter(MAX_ITERATIONS_V))->setMinValue(0);

	VELOCITY_UPDATE_METHOD = createEnumParameter("velocityUpdateMethod", "Velocity update method", &m_velocityUpdateMethod);
	setGroup(VELOCITY_UPDATE_METHOD, "PBD");
	setDescription(VELOCITY_UPDATE_METHOD, "Velocity method.");
	EnumParameter* enumParam = static_cast<EnumParameter*>(getParameter(VELOCITY_UPDATE_METHOD));
	enumParam->addEnumValue("First Order Update", ENUM_VUPDATE_FIRST_ORDER);
	enumParam->addEnumValue("Second Order Update", ENUM_VUPDATE_SECOND_ORDER);
}

void TimeStepController::step(SimulationModel &model)
{
	START_TIMING("simulation step");
	TimeManager *tm = TimeManager::getCurrent ();
	const Real h = tm->getTimeStepSize();
 
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////
	clearAccelerations(model);
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	ParticleData &pd = model.getParticles();
	OrientationData &od = model.getOrientations();

	const int numBodies = (int)rb.size();
	#pragma omp parallel if(numBodies > MIN_PARALLEL_SIZE) default(shared)
	{
		#pragma omp for schedule(static) nowait
		for (int i = 0; i < numBodies; i++)
		{ 
			rb[i]->getLastPosition() = rb[i]->getOldPosition();
			rb[i]->getOldPosition() = rb[i]->getPosition();
			TimeIntegration::semiImplicitEuler(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getVelocity(), rb[i]->getAcceleration());
			rb[i]->getLastRotation() = rb[i]->getOldRotation();
			rb[i]->getOldRotation() = rb[i]->getRotation();
			TimeIntegration::semiImplicitEulerRotation(h, rb[i]->getMass(), rb[i]->getInertiaTensorInverseW(), rb[i]->getRotation(), rb[i]->getAngularVelocity(), rb[i]->getTorque());
			rb[i]->rotationUpdated();
		}

		//////////////////////////////////////////////////////////////////////////
		// particle model
		//////////////////////////////////////////////////////////////////////////
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int) pd.size(); i++)
		{
			pd.getLastPosition(i) = pd.getOldPosition(i);
			pd.getOldPosition(i) = pd.getPosition(i);
			TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
		}

		//////////////////////////////////////////////////////////////////////////
		// orientation model
		//////////////////////////////////////////////////////////////////////////
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int)od.size(); i++)
		{
			od.getLastQuaternion(i) = od.getOldQuaternion(i);
			od.getOldQuaternion(i) = od.getQuaternion(i);
			TimeIntegration::semiImplicitEulerRotation(h, od.getMass(i), od.getInvMass(i) * Matrix3r::Identity() ,od.getQuaternion(i), od.getVelocity(i), Vector3r(0,0,0));
		}
	}

	START_TIMING("position constraints projection");
	positionConstraintProjection(model);
	STOP_TIMING_AVG;
 
	#pragma omp parallel if(numBodies > MIN_PARALLEL_SIZE) default(shared)
	{
		// Update velocities	
		#pragma omp for schedule(static) nowait
		for (int i = 0; i < numBodies; i++)
		{
			if (m_velocityUpdateMethod == 0)
			{
				TimeIntegration::velocityUpdateFirstOrder(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getOldPosition(), rb[i]->getVelocity());
				TimeIntegration::angularVelocityUpdateFirstOrder(h, rb[i]->getMass(), rb[i]->getRotation(), rb[i]->getOldRotation(), rb[i]->getAngularVelocity());
			}
			else
			{
				TimeIntegration::velocityUpdateSecondOrder(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getOldPosition(), rb[i]->getLastPosition(), rb[i]->getVelocity());
				TimeIntegration::angularVelocityUpdateSecondOrder(h, rb[i]->getMass(), rb[i]->getRotation(), rb[i]->getOldRotation(), rb[i]->getLastRotation(), rb[i]->getAngularVelocity());
			}
			// update geometry
			rb[i]->getGeometry().updateMeshTransformation(rb[i]->getPosition(), rb[i]->getRotationMatrix());
		}

		// Update velocities	
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int) pd.size(); i++)
		{
			if (m_velocityUpdateMethod == 0)
				TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
			else
				TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
		}

		// Update velocites of orientations
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int)od.size(); i++)
		{
			if (m_velocityUpdateMethod == 0)
				TimeIntegration::angularVelocityUpdateFirstOrder(h, od.getMass(i), od.getQuaternion(i), od.getOldQuaternion(i), od.getVelocity(i));
			else
				TimeIntegration::angularVelocityUpdateSecondOrder(h, od.getMass(i), od.getQuaternion(i), od.getOldQuaternion(i), od.getLastQuaternion(i), od.getVelocity(i));
		}
	}

	if (m_collisionDetection)
	{
		START_TIMING("collision detection");
		m_collisionDetection->collisionDetection(model);
		STOP_TIMING_AVG;
	}

	velocityConstraintProjection(model);

	//////////////////////////////////////////////////////////////////////////
	// update motor joint targets
	//////////////////////////////////////////////////////////////////////////
	SimulationModel::ConstraintVector &constraints = model.getConstraints();
	for (unsigned int i = 0; i < constraints.size(); i++)
	{
		if ((constraints[i]->getTypeId() == TargetAngleMotorHingeJoint::TYPE_ID) ||
			(constraints[i]->getTypeId() == TargetVelocityMotorHingeJoint::TYPE_ID) ||
			(constraints[i]->getTypeId() == TargetPositionMotorSliderJoint::TYPE_ID) ||
			(constraints[i]->getTypeId() == TargetVelocityMotorSliderJoint::TYPE_ID))
		{
			MotorJoint *motor = (MotorJoint*)constraints[i];
			const std::vector<Real> sequence = motor->getTargetSequence();
			if (sequence.size() > 0)
			{
				Real time = tm->getTime();
				const Real sequenceDuration = sequence[sequence.size() - 2] - sequence[0];
				if (motor->getRepeatSequence())
				{
					while (time > sequenceDuration)
						time -= sequenceDuration;
				}
				unsigned int index = 0;
				while ((2*index < sequence.size()) && (sequence[2 * index] <= time))
					index++;

				// linear interpolation
				Real target = 0.0;
				if (2 * index < sequence.size())
				{
					const Real alpha = (time - sequence[2 * (index - 1)]) / (sequence[2 * index] - sequence[2 * (index - 1)]);
					target = (static_cast<Real>(1.0) - alpha) * sequence[2 * index - 1] + alpha * sequence[2 * index + 1];
				}
				else
					target = sequence[sequence.size() - 1];
				motor->setTarget(target);
			}
		}
	}
	
	// compute new time	
	tm->setTime (tm->getTime () + h);
	STOP_TIMING_AVG;
}

void TimeStepController::reset()
{
	m_iterations = 0;
	m_iterationsV = 0;
	m_maxIterations = 5;
	m_maxIterationsV = 5;
}

void TimeStepController::positionConstraintProjection(SimulationModel &model)
{
	m_iterations = 0;

	// init constraint groups if necessary
	model.initConstraintGroups();
	ParticleData &pd = model.getParticles();
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();
	SimulationModel::ConstraintGroupVector &groups = model.getConstraintGroups();
	SimulationModel::RigidBodyContactConstraintVector &contacts = model.getRigidBodyContactConstraints();
	SimulationModel::ParticleSolidContactConstraintVector &particleTetContacts = model.getParticleSolidContactConstraints();

	// init constraints for this time step if necessary
	for (auto & constraint : constraints)
	{
		constraint->initConstraintBeforeProjection(model);
	}
	control = false;
	
	while (m_iterations < m_maxIterations)
	{
		int nc = 0;
		int c = 0;
		for (unsigned int group = 0; group < groups.size(); group++)
		{
			const int groupSize = (int)groups[group].size();
			#pragma omp parallel if(groupSize > MIN_PARALLEL_SIZE) default(shared)
			{
				#pragma omp for schedule(static) 
				for (int i = 0; i < groupSize; i++)
				{
					const unsigned int constraintIndex = groups[group][i];
					if(constraints[constraintIndex] -> getTypeId() == LineLineConstraint::TYPE_ID)
					{
						int p0 = constraints[constraintIndex] -> m_bodies[0];
						int p1 = constraints[constraintIndex] -> m_bodies[1];
						int p2 = constraints[constraintIndex] -> m_bodies[2];
						int p3 = constraints[constraintIndex] -> m_bodies[3];
						//printf("%d %d %d %d\n", p0, p1, p2, p3);
						if (control == false)
						//if(false)
						{
							pre_solve_edgeedge(model);
							control = true;
							
						}
						if (overlap2[p0][p2] || overlap2[p0][p3] || overlap2[p1][p2] || overlap2[p1][p3])
						{
							constraints[constraintIndex]->updateConstraint(model);
							constraints[constraintIndex]->solvePositionConstraint(model, m_iterations);
					//		printf("Constraint\n");
							c++;
						}
						//else
						{
							nc++;
					//		printf("OK!\n");
						}
					}
			        else {
				    constraints[constraintIndex]->updateConstraint(model);
				    constraints[constraintIndex]->solvePositionConstraint(model, m_iterations);
			        }
				}
			}
		}
		//printf("%d	%d\n", c, c + nc);// << c << ' ' << c + nc << endl;
		for (unsigned int i = 0; i < particleTetContacts.size(); i++)
		{
			particleTetContacts[i].solvePositionConstraint(model, m_iterations);
		}

		m_iterations++;
	}
}


void TimeStepController::velocityConstraintProjection(SimulationModel &model)
{
	m_iterationsV = 0;

	// init constraint groups if necessary
	model.initConstraintGroups();

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();
	SimulationModel::ConstraintGroupVector &groups = model.getConstraintGroups();
	SimulationModel::RigidBodyContactConstraintVector &rigidBodyContacts = model.getRigidBodyContactConstraints();
	SimulationModel::ParticleRigidBodyContactConstraintVector &particleRigidBodyContacts = model.getParticleRigidBodyContactConstraints();
	SimulationModel::ParticleSolidContactConstraintVector &particleTetContacts = model.getParticleSolidContactConstraints();

	for (unsigned int group = 0; group < groups.size(); group++)
	{
		const int groupSize = (int)groups[group].size();
		#pragma omp parallel if(groupSize > MIN_PARALLEL_SIZE) default(shared)
		{
			#pragma omp for schedule(static) 
			for (int i = 0; i < groupSize; i++)
			{
				const unsigned int constraintIndex = groups[group][i];
				constraints[constraintIndex]->updateConstraint(model);
			}
		}
	}

	while (m_iterationsV < m_maxIterationsV)
	{
		for (unsigned int group = 0; group < groups.size(); group++)
		{
			const int groupSize = (int)groups[group].size();
			#pragma omp parallel if(groupSize > MIN_PARALLEL_SIZE) default(shared)
			{
				#pragma omp for schedule(static) 
				for (int i = 0; i < groupSize; i++)
				{
					const unsigned int constraintIndex = groups[group][i];
					constraints[constraintIndex]->solveVelocityConstraint(model, m_iterationsV);
				}
			}
		}

		// solve contacts
		for (unsigned int i = 0; i < rigidBodyContacts.size(); i++)
		{
			rigidBodyContacts[i].solveVelocityConstraint(model, m_iterationsV);
		}
		for (unsigned int i = 0; i < particleRigidBodyContacts.size(); i++)
		{
			particleRigidBodyContacts[i].solveVelocityConstraint(model, m_iterationsV);
		}
		for (unsigned int i = 0; i < particleTetContacts.size(); i++)
		{
			particleTetContacts[i].solveVelocityConstraint(model, m_iterationsV);
		}
		m_iterationsV++;
	}
}



#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Utils/OBJLoader.h"
#include "Demos/Visualization/Visualization.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/TweakBarParameters.h"
#include "Simulation/Simulation.h"

#define _USE_MATH_DEFINES
#include "math.h"


// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void initParameters();
void timeStep();
void buildModel();
void createBunnyRodModel();
void render();
void reset();

DemoBase *base;
DistanceFieldCollisionDetection cd;

// bunny rod scene
const int numberOfBodies = 50;
const Real width = static_cast<Real>(0.1);
const Real height = static_cast<Real>(0.01);
const Real depth = static_cast<Real>(0.01);
const Real youngsModulus = static_cast<Real>(209e9);
const Real torsionModulus = static_cast<Real>(79e9);
const Real density = 7800.;

const Real bunnyDensity = 500.;

void TW_CALL setContactTolerance(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((DistanceFieldCollisionDetection*)clientData)->setTolerance(val);
}

void func_1()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	x -= (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void func_2()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	x += (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void func_3()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	y -= (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void func_4()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	y += (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void func_5()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	z -= (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void func_6()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &pd = model->getRigidBodies();
	Vector3r pos = pd[numberOfBodies]->getPosition();
	//int c = helixRadius;
	double x = pos(0, 0);
	double y = pos(1, 0);
	double z = pos(2, 0);
	z += (width * 3);
	Vector3r newpos = Vector3r(x, y, z);
	pd[numberOfBodies]->setPosition(newpos);
	//pd[numberOfBodies]->setLastPosition(newpos);
	//pd[numberOfBodies]->setOldPosition(newpos);
}
void TW_CALL getContactTolerance(void *value, void *clientData)
{
	*(Real *)(value) = ((DistanceFieldCollisionDetection*)clientData)->getTolerance();
}



// main 
int main(int argc, char **argv)
{
	REPORT_MEMORY_LEAKS

		base = new DemoBase();
	base->init(argc, argv, "Stretch-bending-twisting demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, &cd);
	Simulation::getCurrent()->setModel(model);

	buildModel();

	initParameters();

	Simulation::getCurrent()->setSimulationMethodChangedCallback([&]() { reset(); initParameters(); base->getSceneLoader()->readParameterObject(Simulation::getCurrent()->getTimeStep()); });

	// OpenGL
	MiniGL::setClientIdleFunc(50, timeStep);
	MiniGL::setKeyFunc(0, 'r', reset);

	MiniGL::setKeyFunc(1, '7', func_1);
	MiniGL::setKeyFunc(2, '8', func_2);
	MiniGL::setKeyFunc(3, 'i', func_3);
	MiniGL::setKeyFunc(4, 'u', func_4);
	MiniGL::setKeyFunc(5, '5', func_5);
	MiniGL::setKeyFunc(6, '6', func_6);

	MiniGL::setClientSceneFunc(render);
	MiniGL::setViewport(40.0, 0.1f, 500.0, Vector3r(5.0, 10.0, 30.0), Vector3r(5.0, 0.0, 0.0));

	//TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	glutMainLoop();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void initParameters()
{
	TwRemoveAllVars(MiniGL::getTweakBar());
	TweakBarParameters::cleanup();

	MiniGL::initTweakBarParameters();

	TweakBarParameters::createParameterGUI();
	TweakBarParameters::createParameterObjectGUI(base);
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getModel());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getTimeStep());
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	buildModel();
}

void timeStep()
{
	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	// Simulation code
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;
	}
}

void buildModel()
{
	TimeManager::getCurrent()->setTimeStepSize(static_cast<Real>(0.01));

	createBunnyRodModel();
}

void render()
{
	base->render();
}

void loadObj(const std::string &filename, VertexData &vd, IndexedFaceMesh &mesh, const Vector3r &scale)
{
	std::vector<OBJLoader::Vec3f> x;
	std::vector<OBJLoader::Vec3f> normals;
	std::vector<OBJLoader::Vec2f> texCoords;
	std::vector<MeshFaceIndices> faces;
	OBJLoader::Vec3f s = { (float)scale[0], (float)scale[1], (float)scale[2] };
	OBJLoader::loadObj(filename, &x, &faces, &normals, &texCoords, s);

	mesh.release();
	const unsigned int nPoints = (unsigned int)x.size();
	const unsigned int nFaces = (unsigned int)faces.size();
	const unsigned int nTexCoords = (unsigned int)texCoords.size();
	mesh.initMesh(nPoints, nFaces * 2, nFaces);
	vd.reserve(nPoints);
	for (unsigned int i = 0; i < nPoints; i++)
	{
		vd.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
	}
	for (unsigned int i = 0; i < nTexCoords; i++)
	{
		mesh.addUV(texCoords[i][0], texCoords[i][1]);
	}
	for (unsigned int i = 0; i < nFaces; i++)
	{
		// Reduce the indices by one
		int posIndices[3];
		int texIndices[3];
		for (int j = 0; j < 3; j++)
		{
			posIndices[j] = faces[i].posIndices[j] - 1;
			if (nTexCoords > 0)
			{
				texIndices[j] = faces[i].texIndices[j] - 1;
				mesh.addUVIndex(texIndices[j]);
			}
		}

		mesh.addFace(&posIndices[0]);
	}
	mesh.buildNeighbors();

	mesh.updateNormals(vd, 0);
	mesh.updateVertexNormals(vd);

	LOG_INFO << "Number of triangles: " << nFaces;
	LOG_INFO << "Number of vertices: " << nPoints;
}

/** Create the bunny rod body model
*/
void createBunnyRodModel()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	string fileName = FileSystem::normalizePath(base->getDataPath() + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	loadObj(fileName, vd, mesh, Vector3r(height, width, depth));

	string fileName2 = FileSystem::normalizePath(base->getDataPath() + "/models/suture_needle.obj");
	IndexedFaceMesh mesh2;
	VertexData vd2;
	loadObj(fileName2, vd2, mesh2, Vector3r(0.08, 0.08, 0.08));

	string fileNameSphere = FileSystem::normalizePath(base->getDataPath() + "/models/sphere.obj");
	IndexedFaceMesh meshSphere;
	VertexData vdSphere;
	loadObj(fileNameSphere, vdSphere, meshSphere, Vector3r(0.008, 0.008, 0.008));

	rb.resize(numberOfBodies + 1);
	for (unsigned int i = 0; i < numberOfBodies - 1; i++)
	{
		rb[i] = new RigidBody();

		Real mass(static_cast<Real>(0.25 * M_PI) * width * height * depth * density);

		const Real Iy = static_cast<Real>(1. / 12.) * mass*(static_cast<Real>(3.)*(static_cast<Real>(0.25)*height*height) + width * width);
		const Real Iz = static_cast<Real>(1. / 12.) * mass*(static_cast<Real>(3.)*(static_cast<Real>(0.25)*depth*depth) + width * width);
		const Real Ix = static_cast<Real>(0.25) * mass*(static_cast<Real>(0.25)*(height*height + depth * depth));
		Vector3r inertia(Iy, Ix, Iz); // rod axis along y-axis

		Real angle(static_cast<Real>(M_PI_2));
		Vector3r axis(0., 0., 1.);
		AngleAxisr angleAxis(angle, axis);
		Quaternionr rotation(angleAxis);

		rb[i]->initBody(mass,
			Vector3r((Real)i*width, 0.0, 0.0),
			inertia,
			rotation,
			vd, mesh);
		rb[i]->setRestitutionCoeff(0.05);
		rb[i]->setFrictionCoeff(0.05);
	}
	// Make first body static
	rb[0]->setMass(0.0);
	
	// bunny
	const Quaternionr q(AngleAxisr(static_cast<Real>(1.0 / 6.0*M_PI), Vector3r(0.0, 0.0, 0)));
	const Vector3r t(static_cast<Real>(0.0) + (static_cast<Real>(numberOfBodies - 1.0))*width, static_cast<Real>(0.00)/*-static_cast<Real>(1.776)*/, static_cast<Real>(0.00)/*static_cast<Real>(0.356)*/);
	rb[numberOfBodies - 1] = new RigidBody();
	rb[numberOfBodies - 1]->initBody(bunnyDensity, t, q, vd2, mesh2);
	rb[numberOfBodies - 1]->setMass(1.0);

	const Quaternionr q1(AngleAxisr(static_cast<Real>(1.0 / 6.0*M_PI), Vector3r(0.0, 0.0, 0)));
	const Vector3r t1(static_cast<Real>(0.0) + (static_cast<Real>(numberOfBodies - 1.0))*width - 0.9, -static_cast<Real>(0.08), static_cast<Real>(0.00));
	rb[numberOfBodies] = new RigidBody();
	rb[numberOfBodies]->initBody(bunnyDensity, t1, q1, vdSphere, meshSphere);
	rb[numberOfBodies]->setMass(0.0);

	
	constraints.reserve(3);
	std::vector<std::pair<unsigned int, unsigned int>>  constraintSegmentIndices;
	std::vector<Vector3r> constraintPositions;
	std::vector<Real> averageRadii;
	std::vector<Real> averageSegmentLengths;
	std::vector<Real> youngsModuli(numberOfBodies - 1, youngsModulus);
	std::vector<Real> torsionModuli(numberOfBodies - 1, torsionModulus);

	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, &cd);
	cd.setTolerance(static_cast<Real>(-0.00003));

	for (unsigned int i = 0; i < numberOfBodies - 1; i++)
	{
		const std::vector<Vector3r> *vertices1 = rb[i]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert1 = static_cast<unsigned int>(vertices1->size());
		printf("%d\n", nVert1);
		cd.addCollisionBox(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices1)[0], nVert1, Vector3r(height, width, depth));
	}
	const std::vector<Vector3r> *vertices1 = rb[numberOfBodies - 1]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert1 = static_cast<unsigned int>(vertices1->size());
	//cd.addCollisionObjectWithoutGeometry(numberOfBodies - 1, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices1)[0], nVert1, true);

	/*
	SimulationModel::TetModelVector &tm = model->getTetModels();
	ParticleData &pd = model->getParticles();
	for (unsigned int i = 0; i < tm.size(); i++)
	{
		const unsigned int nVert = tm[i]->getParticleMesh().numVertices();
		unsigned int offset = tm[i]->getIndexOffset();
		tm[i]->setFrictionCoeff(static_cast<Real>(0.1));
		cd.addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
	}*/

	constraintSegmentIndices.reserve(numberOfBodies - 2);
	constraintPositions.reserve(numberOfBodies - 2);
	averageRadii.reserve(numberOfBodies - 2);
	averageSegmentLengths.reserve(numberOfBodies - 2);

	for (unsigned int i = 0; i < numberOfBodies - 2; i++)
	{
		constraintSegmentIndices.push_back(std::pair<unsigned int, unsigned int>(i, i + 1));
		constraintPositions.push_back(Vector3r(static_cast<Real>(i)*width + static_cast<Real>(0.5)*width, 0.0, 0.0));
		averageSegmentLengths.push_back(width);
		averageRadii.push_back(0.25*(height + depth));

	}
	model->addDirectPositionBasedSolverForStiffRodsConstraint(constraintSegmentIndices,
		constraintPositions, averageRadii, averageSegmentLengths, youngsModuli, torsionModuli);

	unsigned int i(numberOfBodies - 2);
	model->addBallJoint(i, i + 1, Vector3r(static_cast<Real>(i)*width + static_cast<Real>(0.5)*width, 0.0, 0.0));

	model->addBallJoint(i + 1, i + 2, Vector3r(static_cast<Real>(i)*width - 0.9, -0.08, 0.0));

	Simulation::getCurrent()->getTimeStep()->setValue(TimeStepController::MAX_ITERATIONS, 200);
}

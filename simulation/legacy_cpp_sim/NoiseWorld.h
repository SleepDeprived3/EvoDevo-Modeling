/*
Based on the Bullet Physics RagdollDemo, modified with help of Josh Bongard's MOOC ludobots

Author: Joshua Hathorne-Madell
*/
#define CONSTRAINT_DEBUG_SIZE 0.2f

#include "GL_ShapeDrawer.h"
#include "GlutDemoApplication.h"
//#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include <string>
//#include <vector>
#include <stdio.h>
#include <math.h>

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

using namespace std;


class BodyPart { 
 public:
  int id;
  btScalar x, y, z;
  btScalar size;
  void readBlueprint(istringstream&);
  void printSelf();
};

void BodyPart::readBlueprint (istringstream &blueprintRow) {
    string val;
    for (int i=0; i<5; i++) {
      getline(blueprintRow, val, ',');
      if (i==0) { id = stoi(val); }
      if (i==1) { x = stof(val); }
      if (i==2) { y = stof(val); }
      if (i==3) { z = stof(val); }
      if (i==4) { size = stof(val); }
    }
}

void BodyPart::printSelf() {
  cout << "Body" << endl;
  cout << id << ", " << x << ", " << y << ", " << z << ", " << size << endl;
}
  

class JointPart {
 public:
  int id, base_body, other_body;
  btScalar ax, ay, az, px, py, pz;
  btScalar lower_limit, upper_limit;
  bool motor;
  void readBlueprint(istringstream&);
  void printSelf();
};

void JointPart::readBlueprint (istringstream &blueprintRow) {
  string val;
  for (int i=0; i<12; i++) {
    getline(blueprintRow, val, ',');
    if (i==0) { id = stoi(val); }
    if (i==1) { base_body = stoi(val); }
    if (i==2) { other_body = stoi(val); }
    if (i==3) { px = stof(val); }
    if (i==4) { py = stof(val); }
    if (i==5) { pz = stof(val); }
    if (i==6) { ax = stof(val); }
    if (i==7) { ay = stof(val); }
    if (i==8) { az = stof(val); }
    if (i==9) { lower_limit = stof(val); }
    if (i==10) { upper_limit = stof(val); }
    if (i==11) { 
      if (val[0]=='T') { motor = true; }
      if (val[0]=='F') { motor = false; }
    }
  }
}

void JointPart::printSelf() {
  cout << "Joint" << endl;
  cout << id << ", " << base_body << ", " << other_body << endl;
  cout << px << ", "<< py << ", "<< pz << endl;
  cout << ax << ", " << ay << ", " << az << endl;
  cout << lower_limit << ", " << upper_limit << ", " << motor << endl;
}


class SensorPart
{
 public:
  int id, body_id;
  btScalar x, y, z;
  void readBlueprint(istringstream&);
  void printSelf();
};

void SensorPart::readBlueprint (istringstream &blueprintRow) {
  string val;
  for (int i=0; i<5; i++) {
    getline(blueprintRow, val, ',');
    if (i==0) { id = stoi(val); }
    if (i==1) { body_id = stoi(val); }
    if (i==3) { x = stof(val); }
    if (i==3) { y = stof(val); }
    if (i==4) { z = stof(val); }
  }
}

void SensorPart::printSelf() {
  cout << "Sensor" << endl;
  cout << id << ", " << body_id << ", " << x << ", " << y << ", " << z << endl;
}


class NoiseWorld : public GlutDemoApplication
{
	// ---Basic BP components---
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionShape* groundShape;	
	btCollisionObject* fixedGround;
	// ---Control variables---
	bool oneStep;
	bool pause;
	btScalar stepTime = btScalar(1.)/btScalar(60.);

	// ---Bullet Physics constructs for robot parts---
	btAlignedObjectArray<btRigidBody*> m_bodyparts;
	btAlignedObjectArray<btCollisionShape*> m_geomparts;
	btAlignedObjectArray<btHingeConstraint*> m_jointparts;
	// don't need \/ this \/ one because of m_geomparts probably
	// btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	// ---Sensor_touches variables---
	int sensor_body_id;
	int sensor_touches_id;
	btScalar body_size;
	const btScalar sensor_radius = .1;
	btScalar bp_x;
	btScalar bp_y;
	btScalar bp_z;
	btScalar sp_x;
	btScalar sp_y;
	btScalar sp_z;
	btScalar t_x;
	btScalar t_y;
	btScalar t_z;
	// ---ANN outputs---
	btAlignedObjectArray<btScalar> output_s2n;
	btAlignedObjectArray<btScalar> output_n2n;
	btAlignedObjectArray<btScalar> output_s2j;
	btAlignedObjectArray<btScalar> output_n2j;
	btScalar motor_command;
 public:
	// ---Blueprint Variable---
	int simulation_number;
	char io_file[80];
	btScalar gravity_value = -9.81;
	btAlignedObjectArray<BodyPart> bodys;
	btAlignedObjectArray<JointPart> joints;
	btAlignedObjectArray<SensorPart> sensors;
	// ---ANN Matrices---
	btAlignedObjectArray<btAlignedObjectArray<btScalar> > weights_s2n;
	btAlignedObjectArray<btAlignedObjectArray<btScalar> > weights_n2n;	
	btAlignedObjectArray<btAlignedObjectArray<btScalar> > weights_s2j;
	btAlignedObjectArray<btAlignedObjectArray<btScalar> > weights_n2j;
	// ---Collision variables---
	//int * IDs;
	btAlignedObjectArray<int*> IDs;
 	btAlignedObjectArray<int> bodyTouches;
	btAlignedObjectArray<btScalar> sensorTouches;
	btAlignedObjectArray<btVector3> touchesPoint;
	// ---Display Variables---
	bool drawGraphics; 
	unsigned long int timeStep;

	// Methods
	btAlignedObjectArray<BodyPart> UseBodyBlueprints(string filename)
	{
	  btAlignedObjectArray<BodyPart> bodyList;
	  string line;
	  ifstream blueprintFile(filename);
	  while (getline(blueprintFile, line))
	    {
	      istringstream constructInfo(line);
	      BodyPart part;
	      part.readBlueprint(constructInfo);
	      bodyList.push_back(part);
	    }
	  blueprintFile.close();
	  return bodyList;
	}


	btAlignedObjectArray<JointPart> UseJointBlueprints(string filename)
	{
	  btAlignedObjectArray<JointPart> jointList;
	  string line;
	  ifstream blueprintFile(filename);
	  while (getline(blueprintFile, line))
	    {
	      istringstream constructInfo(line);
	      JointPart part;
	      part.readBlueprint(constructInfo);
	      jointList.push_back(part);
	    }
	  blueprintFile.close();
	  return jointList;
	}
	  
	
	btAlignedObjectArray<SensorPart> UseSensorBlueprints(string filename)
	  {
	    btAlignedObjectArray<SensorPart> sensorList;
	    string line;
	    ifstream blueprintFile(filename);
	    while (getline(blueprintFile, line))
	      {
		istringstream constructInfo(line);
		SensorPart part;
		part.readBlueprint(constructInfo);
		sensorList.push_back(part);
	      }
	    blueprintFile.close();
	    return sensorList;
	  }

	
	btAlignedObjectArray<btAlignedObjectArray<btScalar > > UseMatrixBlueprints(string filename)
	  {
	    btAlignedObjectArray<btAlignedObjectArray<btScalar > > matrix;
	    btAlignedObjectArray<btScalar> row;
	    string line;
	    string element;
	    ifstream blueprintFile(filename);
	    while (getline(blueprintFile, line))
	      {
		istringstream lineStream(line);
		while (getline(lineStream, element, ','))
		  {
		    row.push_back(stof(element));
		  }
		matrix.push_back(row);
		row.resize(0);
	      }
	    return matrix;
	  }
		  
	  
	// building functions
	void CreateGround(int index)
	{
	  groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	  //m_collisionShapes.push_back(groundShape);
	  m_geomparts.push_back(groundShape);
	  
	  btTransform groundTransform;
	  groundTransform.setIdentity();
	  groundTransform.setOrigin(btVector3(0,-1,0));
	  
	  fixedGround = new btCollisionObject();
	  fixedGround->setCollisionShape(groundShape);
	  fixedGround->setWorldTransform(groundTransform);
	  fixedGround->setUserPointer(&(IDs[index]));
	  m_dynamicsWorld->addCollisionObject(fixedGround);  
	}
	

	void CreateSphere(int index, btScalar x, btScalar y, btScalar z, btScalar size)
	{
	  btCollisionShape* sphere = new btSphereShape(size);
	  //m_collisionShapes.push_back(sphere);
	  m_geomparts.push_back(sphere);
	  btTransform sphereTransform;
	  sphereTransform.setIdentity();
	  btVector3 position (x, y, z);
	  sphereTransform.setOrigin(position);
	  btScalar volume = (4/3) * size * size * size * 3.14159265359;
	  btScalar density = 1.9098593171;
	  btScalar mass = volume * density;
	  bool isDynamic = (mass != 0.f);
	  btVector3 localInertia(0,0,0);
	  if (isDynamic)
	    {
	      sphere->calculateLocalInertia(mass, localInertia);
	      sphereTransform.setOrigin(position);
	      btDefaultMotionState* myMotionState = new btDefaultMotionState(sphereTransform);
	      btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
	      m_bodyparts.push_back(new btRigidBody(rbInfo));
	      m_bodyparts[index]->setUserPointer(&(IDs[index+1])); 
	      m_bodyparts[index]->setFriction(0.8);
	      m_bodyparts[index]->setRollingFriction(0.5);
	      m_bodyparts[index]->setActivationState(DISABLE_DEACTIVATION);
	      m_dynamicsWorld->addRigidBody(m_bodyparts[index]);
	    }
	}
     

	btVector3 PointWorldToLocal(int bodyIndex, btVector3 &point)
	{
	  return m_bodyparts[bodyIndex]->getCenterOfMassTransform().inverse()(point);
	}


	btVector3 AxisWorldToLocal(int bodyIndex, btVector3 &axis)
	{
	  btTransform local1 =  m_bodyparts[bodyIndex]->getCenterOfMassTransform().inverse();
	  btVector3 myZero(0., 0., 0.);
	  local1.setOrigin(myZero);
	  return local1 * axis;
	}


	void CreateHinge(int index, int body1, int body2,
			 btScalar px, btScalar py, btScalar pz,
			 btScalar ax, btScalar ay, btScalar az,
			 btScalar lower, btScalar upper, bool motor)
	{
	  btVector3 position (px, py, pz);
	  btVector3 axis (ax, ay, az);
	  btVector3 locPoint1 = PointWorldToLocal(body1, position);
	  btVector3 locAxis1 = AxisWorldToLocal(body1, axis);
	  btVector3 locPoint2 = PointWorldToLocal(body2, position);
	  btVector3 locAxis2 = AxisWorldToLocal(body2, axis);
	  btHingeConstraint* joint = new btHingeConstraint(*m_bodyparts[body1], *m_bodyparts[body2], 
							   locPoint1, locPoint2, locAxis1, locAxis2,
							   true);
	  joint->setLimit(lower, upper);
	  joint->enableMotor(motor);
	  joint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	  m_jointparts.push_back(joint);
	  m_dynamicsWorld->addConstraint(m_jointparts[index], false);
	  
	}
	

	void ActuateJoint(int jointIndex, btScalar desiredAngle, btScalar dt)
	{
	  m_jointparts[jointIndex]->setMaxMotorImpulse(.4);
	  m_jointparts[jointIndex]->setMotorTarget(desiredAngle, dt);
	}
	

	void DeleteGround()
	{
	  delete groundShape;
	  delete fixedGround;
	}


	void DeleteObject(int index)
	{
	  delete m_geomparts[index+1];
	  delete m_bodyparts[index]->getMotionState();
	  m_dynamicsWorld->removeCollisionObject(m_bodyparts[index]);
	  delete m_bodyparts[index];
	}


	void DeleteHinge(int index)
	{
	  delete m_jointparts[index];
	}


	btAlignedObjectArray<btScalar> CalculateLayer(btAlignedObjectArray<
						      btAlignedObjectArray<btScalar> > matrix, 
						      btAlignedObjectArray<btScalar> dataIn)
	  {
	    btScalar d_hold (0);
	    btAlignedObjectArray<btScalar> output;
	    for (int j=0; j<matrix[0].size(); j++)
	      {
		d_hold = 0;
		for (int i=0; i<dataIn.size(); i++)
		  {
		    d_hold += dataIn[i] * matrix[i][j];
		  }
		output.push_back(d_hold);
	      }
	    return output;
	  }
	

	int Save_Position(bool completed)
	{
	  btVector3 position = m_bodyparts[0]->getCenterOfMassTransform().getOrigin();
	  btScalar distance = 0;
	  if (completed)
	    {
	      distance = sqrt(pow(position.getX(), 2) + pow(position.getZ(), 2));
	    }
	  string sim_str = to_string(simulation_number);
	  string extension (".dat");
	  int index;
	  string fitsFileName ("sim_.dat");
	  fitsFileName.insert(0, io_file);
	  index = fitsFileName.find(extension);
	  fitsFileName.insert(index, sim_str);
	  ofstream fitsFile(fitsFileName, ios::out);
	  fitsFile << distance << endl;
	  fitsFile.close();
	  //exit(0);
	  //exitPhysics();
	  return 0;
	}

	void initPhysics();

	void exitPhysics();

	void ParseCmdLine(char * swtxt);
	
	virtual ~NoiseWorld()
	{
	  exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);	
	
	static DemoApplication* Create()
	{
		NoiseWorld* demo = new NoiseWorld;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}	
};
      

  
  

#include <ode\ode.h>
#include <drawstuff\drawstuff.h>

//To allow for conversion from dReal to float
#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define NUM 4

dWorldID world;
dBodyID link[NUM];
dJointID joint[NUM]; //to move the arm we only need 3 joints but to fix the arm on the ground add a fixed joint between the base and the ground

static double THETA[NUM] = {0.0, 0.0, 0.0, 0.0}; //Target joint angles
static double l[NUM] = {0.10, 0.90, 1.00, 1.00}; //The length of the links
static double r[NUM] = {0.20, 0.04, 0.04, 0.04}; //The radius of the links

void control() //Function implementing joint proportional controller
{
	static int step = 0;
	double k1 = 10.0, fMax = 100.0;

	printf("\r%6d:", step++);
	for(int j = 1; j < NUM; ++j)
	{
		double tmpAngle = dJointGetHingeAngle(joint[j]);
		double z = THETA[j] - tmpAngle; //calculating delta theta
		printf(" delta change %d for joint %i\n",z, j);
		dJointSetHingeParam(joint[j], dParamVel, k1*z); //Set the angular velocity of the joint
		dJointSetHingeParam(joint[j], dParamFMax, fMax); //Set the maximum force that can be applied to the joint
	}

}

void start()
{
	float xyz[3] = {3.04, 1.28, 0.76}; //View point x, y, z
	float hpr[3] = {-160.0, 4.5, 0.00}; //View direction heading, pitch, roll
	dsSetViewpoint(xyz, hpr);
}

void command(int cmd) //Get input from the keyboard
{
	switch (cmd)
	{
		case 'a': 
			THETA[1] += 0.05; 
			break;
		case 's':
			THETA[1] -= 0.05;
			break;
		case 'd':
			THETA[2] += 0.05;
			break;
		case 'f':
			THETA[2] -= 0.05;
			break;
		case 'g':
			THETA[3] += 0.05;
			break;
		case 'h':
			THETA[3] -= 0.05;
			break;
		default:
			break;
	}

	if(THETA[1] < -M_PI) 
		THETA[1] = -M_PI;
	if(THETA[1] >= M_PI) //There is an instability when the joint reaches 2 PI angle ! it just goes crazy!
		THETA[1] = M_PI;
	if(THETA[2] < -2*M_PI/3)
		THETA[2] = -2*M_PI/3;
	if(THETA[2] > 2*M_PI/3)
		THETA[2] = 2*M_PI/3;
	if(THETA[3] < -2*M_PI/3)
		THETA[3] = -2*M_PI/3;
	if(THETA[3] > 2*M_PI/3)
		THETA[3] = 2*M_PI/3;
}

void simLoop (int pause)
{
	control();
	dWorldStep(world, 0.01);

	dsSetColor(1.0, 1.0, 1.0);

	for(int i = 0; i < NUM; ++i)
	{
		dsDrawCapsuleD(dBodyGetPosition(link[i]), dBodyGetRotation(link[i]), l[i], r[i]);
	}
}

int main(int argc, char **argv)
{
	dsFunctions fn;
	//Setting the center of gravity of each link
	double x[NUM] = {0.0}, y[NUM] = {0.0};
	double z[NUM] = {0.05, 0.55, 1.55, 2.55};
	//Setting the mass of each link
	double m[NUM] = {10.00, 2.00, 2.00, 2.00};
	//Anchor define the position where the joint is going to be attached
	double anchor_X[NUM] = {0.0}, anchor_Y[NUM] = {0.0};
	double anchor_Z[NUM] = {0.00, 0.10, 1.00, 2.00};
	//Setting the axes of the hinge joints .. each joint can only rotate about one axis!
	double axis_X[NUM] = {0.00, 0.00, 0.00, 0.00};
	double axis_Y[NUM] = {0.00, 0.00, 1.00, 1.00};
	double axis_Z[NUM] = {1.00, 1.00, 0.00, 0.00};

	//Setting the draw functions
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "../Debug/textures";

	dInitODE();
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);

	//Create the link bodies in the world and set their mass
	for(int i = 0; i < NUM; ++i)
	{
		dMass mass;
		link[i] = dBodyCreate(world);
		dBodySetPosition(link[i], x[i], y[i], z[i]);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, m[i], 3, r[i], l[i]);
		dBodySetMass(link[i], &mass);
	}

	//Create the joints and attach them to the links
	joint[0] = dJointCreateFixed(world, 0); //Create fixed joint to attach the base of the arm to the ground
	dJointAttach(joint[0], link[0], 0);
	dJointSetFixed(joint[0]);

	for (int j = 1; j < NUM; ++j)
	{
		joint[j] = dJointCreateHinge(world, 0);
		dJointAttach(joint[j], link[j-1], link[j]);
		dJointSetHingeAnchor(joint[j], anchor_X[j], anchor_Y[j], anchor_Z[j]);
		dJointSetHingeAxis(joint[j], axis_X[j], axis_Y[j], axis_Z[j]);
	}

	dsSimulationLoop(argc, argv, 640, 570, &fn);
	dWorldDestroy(world);
	dCloseODE();
	return 0;
}
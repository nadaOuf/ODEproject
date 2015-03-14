#include "AbstractSkeleton.h"
#include "Trajectories.h"

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox	dsDrawBoxD
#define sdDrawSphere dsDrawSphereD
#endif

#define LINK_NUM 3	//Number of links per leg
#define JOINT_NUM 3	//Number of joints per leg
#define LEG_NUM 4	//Number of legs

//world to add the bodies in 
dWorldID world;

//for collisions
dSpaceID space; 
dGeomID ground;
dJointGroupID contactgroup;

//Functions for drawing
dsFunctions fn;

Link* root;

dReal THETA[LEG_NUM][LINK_NUM] = {{0},{0},{0},{0}};

float T = 7;
float phase;
dReal legFrameTorques[2][2][3][3]; //2 leg frames each has 2 legs each is composed of 3 joints with 3 DOFs
AbstractSkeleton* skeleton;
Trajectories* trajectories;
int step = 0;

void setupSkeleton()
{
	dMass mass;
	dReal height = 0.47;
	dReal linkRadius = 0.05;
	dReal linkLength = (height-(linkRadius + 0.01))/2.0 - linkRadius;
	dReal linkMass = 1;

	root  = new Link(NULL);
	Link torso = *root;
	//Set up the torso
	torso.body = dBodyCreate(world);
	torso.radius = 0.2;
	torso.length = 0.5;
	torso.mass = 22;
	
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, torso.mass, 3, torso.radius, torso.length);
	dBodySetMass(torso.body, &mass);

	torso.cX = 0;
	torso.cY = 0;
	torso.cZ = height;
	dBodySetPosition(torso.body, torso.cX, torso.cY, torso.cZ);
	dMatrix3 Rotation;
	dRFromAxisAndAngle(Rotation, 0, 1, 0, M_PI/2);
	dBodySetRotation(torso.body, Rotation);	
	
	torso.geom = dCreateCapsule(space, torso.radius, torso.length);
	dGeomSetBody(torso.geom, torso.body);

	torso.AddChild(new Link(root));
	torso.AddChild(new Link(root));
	torso.AddChild(new Link(root));
	torso.AddChild(new Link(root));

	//Set up the links
	int altX = -1;
	int altY = -1;
	Link* parent;
	for (int i = 0; i < LEG_NUM; ++i)
	{
		parent = torso.GetChild(i);
		dReal x = torso.length/2.0;
		dReal y = torso.radius;
		for (int j = 0; j < LINK_NUM; ++j)
		{
			Link* link = new Link(parent);
			link->body = dBodyCreate(world);
			link->radius = linkRadius;
			if(j == LINK_NUM - 1)
				link->length = 0.03;
			else
				link->length = linkLength;

			link->mass = linkMass;
			dMassSetZero(&mass);
			
			if (j == LINK_NUM - 1)
			{
				link->totalLength = link->length + linkRadius;
				dMassSetBoxTotal(&mass, link->mass, link->radius*2, link->radius*2, link->length);
				link->geom = dCreateBox(space, link->radius*2, link->radius*2, link->length);
				//x += leg[i][j].radius*altX;
			}
			else
			{
				link->totalLength = link->length + linkRadius;							
				dMassSetCapsuleTotal(&mass, link->mass, 3, link->radius, link->length);
				link->geom = dCreateCapsule(space, link->radius, link->length);
			}

			dBodySetMass(link->body, &mass);
			dGeomSetBody(link->geom, link->body);

			//Setting the position of the link
			link->cX = x*altX;
			link->cY = y*altY;
			link->cZ = height - link->totalLength/2.0 - (j > 0? link->totalLength *j : 0);
			
			dBodySetPosition(link->body, link->cX, link->cY, link->cZ);	
			parent = link;
		}

		if (i == 1)
			altX += 2;

		altY += 2;
		if (altY > 1)
			altY = -1;
	}

	torso.AddChild(new Link(root));
	Link head = torso.GetChild(4);
	//Add the head
	head.body = dBodyCreate(world);
	head.length = 0.3;
	head.radius = 0.1;
	head.mass = 1;
	head.cX = torso.length/2.0 + head.length/2.0;
	head.cY = 0;
	head.cZ = torso.cZ + torso.radius*1.5;

	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, head.mass, 3, head.radius, head.length);
	dBodySetMass(head.body, &mass);

	dBodySetPosition(head.body, head.cX, head.cY, head.cZ);
	dBodySetRotation(head.body, Rotation);

	head.geom = dCreateCapsule(space, head.radius, head.length);
	dGeomSetBody(head.geom, head.body);

	//Connect the joints
	//Attach the head and torso
	head.joint = dJointCreateHinge(world, 0);
	dJointAttach(head.joint, head.body, torso.body);
	dJointSetHingeAnchor(head.joint, head.cX, head.cY, head.cZ);
	dJointSetHingeAxis(head.joint, 0,1,0);

	dReal stop = 3.14f / 2.0f;
	dReal fmax = 10.0f;
	dReal cfm = 1e-5;
	dReal erp = 0.8f;
	Link* link;
	//Attach the link joints
	for(int i = 0; i < LEG_NUM; ++i)
	{
		parent = torso.GetChild(i);
		for(int j = 0; j < JOINT_NUM; ++j)
		{
			link = parent;
			
			switch(j)
			{
				case 0: //Create a ball-in-socket joint and connect to the torso
					link->joint = dJointCreateBall(world, 0);
					dJointAttach(link->joint, link->body, link->GetParent()->body);
					dJointSetBallAnchor(link->joint, link->cX, link->cY, link->cZ + link->totalLength/2.0);

					//For a ball-in-socket joint create a AMotor to control it
					link->aMotor = dJointCreateAMotor(world, 0);
					dJointAttach(link->aMotor, link->body, link->GetParent()->body);
					dJointSetAMotorMode(link->aMotor, dAMotorUser);
					dJointSetAMotorNumAxes(link->aMotor, 3);
					dJointSetAMotorAxis(link->aMotor, 0, 1,  0, 0, 1);
					dJointSetAMotorAxis(link->aMotor, 1, 1,  1, 0, 0);
					dJointSetAMotorAxis(link->aMotor, 2, 1,  0, 1, 0);

					dJointSetAMotorParam(link->aMotor, dParamLoStop,  -stop);
					dJointSetAMotorParam(link->aMotor, dParamHiStop,   stop);
					dJointSetAMotorParam(link->aMotor, dParamVel,      0);
					dJointSetAMotorParam(link->aMotor, dParamFMax,     fmax);
					dJointSetAMotorParam(link->aMotor, dParamBounce,   0);
					dJointSetAMotorParam(link->aMotor, dParamStopCFM,  cfm);
					dJointSetAMotorParam(link->aMotor, dParamStopERP,  erp);

					//Hinge joint for test
					/*leg[i][j].joint = dJointCreateHinge(world, 0);
					dJointAttach(leg[i][j].joint, leg[i][j].body, torso.body);
					dJointSetHingeAnchor(leg[i][j].joint, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ + leg[i][j].totalLength/2.0);
					dJointSetHingeAxis(leg[i][j].joint, 0, 1, 0);*/
					break;
				case 1: //Create a hinge joint and connect to the previous link
					link->joint = dJointCreateHinge(world, 0);
					dJointAttach(link->joint, link->body, link->GetParent()->body);
					dJointSetHingeAnchor(link->joint, link->cX, link->cY, link->cZ + link->totalLength/2.0);
					dJointSetHingeAxis(link->joint, 0, 1, 0);
					break;
				case 2: //Create a universal joint and connect to the previous link
					link->joint = dJointCreateUniversal(world, 0);
					dJointAttach(link->joint, link->body, link->GetParent()->body);
					dJointSetUniversalAnchor(link->joint, link->cX, link->cY, link->cZ + link->totalLength/2.0);
					dJointSetUniversalAxis1(link->joint, 0, 1, 0);
					dJointSetUniversalAxis2(link->joint, 0, 0, 1);
					break;
				default:
					break;
			}

			//Create the joint Controller
			link->PD = new Controller(50, 5);
			parent = link->GetChild();
		}

	}

	skeleton = new AbstractSkeleton();
	IKChain* FR = new IKChain(root->GetChild(2), root->GetChild(2)->GetChild()->GetChild(), 2);
	IKChain* FL = new IKChain(root->GetChild(3), root->GetChild(3)->GetChild()->GetChild(), 2);
	LegFrame* shoulder = new LegFrame(FL, FR);
	IKChain* RR = new IKChain(root->GetChild(0), root->GetChild(0)->GetChild()->GetChild(), 2);
	IKChain* RL = new IKChain(root->GetChild(1), root->GetChild(1)->GetChild()->GetChild(), 2);
	LegFrame* hip = new LegFrame(RL, RR);
	
	skeleton->setSkeletonRoot(root);	

	skeleton->setLegFrames(shoulder, hip);

}

void drawSkeleton ()
{
	//draw the torso
	dsSetColor(1.3, 1.3, 1.3);
	dsDrawCapsule(dBodyGetPosition(root->body), dBodyGetRotation(root->body), root->length, root->radius);

	//draw links
	LegFrame* drawFrame;
	Link* parent;
	Link* link;
	for(int legFrame = 0; legFrame < 2; ++legFrame)
	{
		drawFrame = skeleton->getLegFrame(legFrame);
		for(int i = 0; i < 2; ++i)
		{
			parent = i == 0? drawFrame->getLeftRoot(): drawFrame->getRightRoot();
			for (int j = 0; j < LINK_NUM; ++j)
			{
				link = parent;
				if(j == LINK_NUM-1)
				{
					dReal dim[3] = {link->radius*2, link->radius*2, link->length};
					dsDrawBox(dBodyGetPosition(link->body), dBodyGetRotation(link->body), dim);
				}
				else
					dsDrawCapsule(dBodyGetPosition(link->body), dBodyGetRotation(link->body), link->length, link->radius);
				
				parent = parent->GetChild();
			}
		}
	}

	dsDrawCapsule(dBodyGetPosition(root->GetChild(4)->body), dBodyGetRotation(root->GetChild(4)->body), root->GetChild(4)->length, root->GetChild(4)->radius);
}

void createTrajectories()
{
	trajectories = new Trajectories();
	for(int i = 0; i < LEG_NUM; ++i)
	{
		vector<glm::vec3> walkPoints;
		float stepSize = 0.4;
		float height = 0.2;
		switch(i)
		{
			case 0: //RL
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize/2.0f, 0, height));
				walkPoints.push_back(glm::vec3(3.0f*stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));
				trajectories->generateLegTrajectory(trajectories->RL, walkPoints);
				break;
			case 1: //RR
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize/2.0f, 0, height));
				walkPoints.push_back(glm::vec3(3.0f*stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, height/2.0f));
				trajectories->generateLegTrajectory(trajectories->RR, walkPoints);
				break;
			case 2: //FL
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, 0));
				walkPoints.push_back(glm::vec3(0, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize/2.0f, 0, height));
				walkPoints.push_back(glm::vec3(3.0f*stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));
				trajectories->generateLegTrajectory(trajectories->FL, walkPoints);
				break;
			case 3: //FR
				walkPoints.push_back(glm::vec3(0, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize/2.0f, 0, height));
				walkPoints.push_back(glm::vec3(3.0f*stepSize/4.0f, 0, height*3.0f/4.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, height/2.0f));
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));				
				walkPoints.push_back(glm::vec3(stepSize, 0, 0));
				trajectories->generateLegTrajectory(trajectories->FR, walkPoints);
				break;
			default:
				break;
		}
	}

}

void calculateTargetAngles()
{
	
}

void ComputePDTorques()
{
	dReal maxF = 100.0;
	LegFrame* legframe;
	Link* link;
	for(int x = 0; x < 2; ++x)
	{
		legframe = skeleton->getLegFrame(x);
		for (int i = 0; i < 2; ++i)
		{
			link = i == 0? legframe->getLeftRoot() : legframe->getRightRoot();
			for (int j = 0; j < LINK_NUM; ++j)
			{
				glm::vec3 angles = glm::vec3(0, 0, 0);
				glm::vec3 omega = glm::vec3(0, 0, 0);
				glm::vec3 result = glm::vec3(0, 0, 0);
				glm::vec3 delta = glm::vec3(0, 0, 0);
				switch(j)
				{
					case 0: //ball-and-socket joint
						angles[2] = dJointGetAMotorAngle(link->aMotor, 0);
						angles[0] = dJointGetAMotorAngle(link->aMotor, 1);
						angles[1] = dJointGetAMotorAngle(link->aMotor, 2);

						omega[2] = dJointGetAMotorAngleRate(link->aMotor, 0);
						omega[0] = dJointGetAMotorAngleRate(link->aMotor, 1);
						omega[1] = dJointGetAMotorAngleRate(link->aMotor, 2);

						delta = glm::vec3(link->tX - angles[0], link->tY - angles[1], link->tZ - angles[2]);
						result = link->PD->calculateControllerOutput(delta, omega);
						//printf("motor torques %d %d %d\n", result.x, result.y, result.z);
						legFrameTorques[x][i][j][0] = result.z;
						legFrameTorques[x][i][j][1] = result.x;
						legFrameTorques[x][i][j][2] = result.y;
						//dJointAddAMotorTorques(link->aMotor, result.z, result.x, result.y);
					
						break;
					case 1: //hinge joint
						angles[1] = dJointGetHingeAngle(link->joint);
						omega[1] = dJointGetHingeAngleRate(link->joint);
						omega[2] = 0;
						delta = glm::vec3(link->tX - angles[0], link->tY - angles[1], link->tZ - angles[2]);
						//printf("omega %d %d %d\n", omega[0], omega[1], omega[2]);
						result = link->PD->calculateControllerOutput(delta, omega);
						legFrameTorques[x][i][j][0] = 0;
						legFrameTorques[x][i][j][1] = result.y;
						legFrameTorques[x][i][j][2] = 0;
						//printf("result %d %d %d\n", result.x, result.y, result.z);
						//dJointAddHingeTorque(link->joint, result[1]);
						//dJointSetHingeParam(link->joint, dParamFMax, maxF);
						break;
					case 2: //universal joint
						angles[1] = dJointGetUniversalAngle1(link->joint);
						angles[2] = dJointGetUniversalAngle2(link->joint);
						omega[1] = dJointGetUniversalAngle1Rate(link->joint);
						omega[2] = dJointGetUniversalAngle2Rate(link->joint);
						delta = glm::vec3(link->tX - angles[0], link->tY - angles[1], link->tZ - angles[2]);
						result = link->PD->calculateControllerOutput(delta, omega);
						legFrameTorques[x][i][j][0] = 0;
						legFrameTorques[x][i][j][1] = result.y;
						legFrameTorques[x][i][j][2] = result.z;
						//dJointAddUniversalTorques(link->joint, result[1], 0);
						//dJointSetUniversalParam(link->joint, dParamFMax, maxF);
						//dJointSetUniversalParam(link->joint, dParamFMax1, maxF);
						break;
				}
			}
			link = link->GetChild();
		}
	}
}

void ApplyLegFrameTorques()
{
	
}

void setupWorld ()
{
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground = dCreatePlane(space, 0, 0, 1, 0);

	dWorldSetGravity(world, 0, 0, -9.8);
	dWorldSetCFM(world, 0);
	dWorldSetERP(world, 0);
}

//Start function that will be called by the draw functions
void start ()
{
	float xyz[3] = {  2.0f,  -2.0f, 1.0f};
	float hpr[3] = {121.0f, -10.0f, 0.0f};
	dsSetViewpoint(xyz,hpr);
	dsSetCapsuleQuality(6);
}

//Function to detect potential collision between two objects o1 and o2
//If there is a collision a joint is attached between the two objects ?
void nearCollide (void *data, dGeomID o1, dGeomID o2)
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) //Check if the two bodies are already connectedby non-contact joints
		return;

	const int N = 20; //Maximum number of contacts
	dContact contact[N];
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if (n > 0)
		for (int i = 0; i < n; ++i)
		{
			contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
			contact[i].surface.mu   = dInfinity; //2.0;
			contact[i].surface.soft_erp = 0;
			contact[i].surface.soft_cfm = 0;//1e-5;
			dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach(c,b1,b2);
		}
}

//Simulation function that will be called each time step
//Drawing function has to be in the simulation loop
void simLoop (int pause)
{

	//Calculate Torques
	calculateTargetAngles();
	ComputePDTorques();

	//Apply torques to legframes
	ApplyLegFrameTorques();

	//world step
	dSpaceCollide(space, 0, &nearCollide);
	dWorldStep(world, 0.0001);
	dJointGroupEmpty(contactgroup);

	drawSkeleton();

	phase += 1;

	if(phase >= T)
		phase -= T;
}

void initDraw ()
{
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = & simLoop;
	fn.command = 0;
	fn.stop = 0;
	fn.path_to_textures = "../Debug/textures";
}

int main (int argc, char **argv)
{
	//Initialize functions
	initDraw ();
	dInitODE ();

	setupWorld ();
	setupSkeleton ();

	createTrajectories();

	dsSimulationLoop (argc, argv, 800, 480, &fn);

	dWorldDestroy (world);
	dCloseODE ();
	return 0;
}




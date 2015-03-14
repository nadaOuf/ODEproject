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

MyLink leg[LEG_NUM][LINK_NUM], torso, head;

dReal THETA[LEG_NUM][LINK_NUM] = {{0},{0},{0},{0}};

float T = 7;
float phase;
dReal legFrameTorques[2][2][3][3]; //2 leg frames each has 2 legs each is composed of 3 joints with 3 DOFs
AbstractSkeleton* skeleton;
Trajectories* trajectories;
int step = 0;

void setupSkeleton ()
{
	dMass mass;
	dReal height = 0.47;
	dReal linkRadius = 0.05;
	dReal linkLength = (height-(linkRadius + 0.01))/2.0 - linkRadius;
	dReal linkMass = 1;

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

	//Set up the links
	int altX = -1;
	int altY = -1;
	for (int i = 0; i < LEG_NUM; ++i)
	{
		dReal x = torso.length/2.0;
		dReal y = torso.radius;
		for (int j = 0; j < LINK_NUM; ++j)
		{
			leg[i][j].body = dBodyCreate(world);
			leg[i][j].radius = linkRadius;
			if(j == LINK_NUM - 1)
				leg[i][j].length = 0.03;
			else
				leg[i][j].length = linkLength;

			leg[i][j].mass = linkMass;
			dMassSetZero(&mass);
			
			if (j == LINK_NUM - 1)
			{
				leg[i][j].totalLength = leg[i][j].length + linkRadius;
				dMassSetBoxTotal(&mass, leg[i][j].mass, leg[i][j].radius*2, leg[i][j].radius*2, leg[i][j].length);
				leg[i][j].geom = dCreateBox(space, leg[i][j].radius*2, leg[i][j].radius*2, leg[i][j].length);
				//x += leg[i][j].radius*altX;
			}
			else
			{
				leg[i][j].totalLength = leg[i][j].length + linkRadius;							
				dMassSetCapsuleTotal(&mass, leg[i][j].mass, 3, leg[i][j].radius, leg[i][j].length);
				leg[i][j].geom = dCreateCapsule(space, leg[i][j].radius, leg[i][j].length);
			}

			dBodySetMass(leg[i][j].body, &mass);
			dGeomSetBody(leg[i][j].geom, leg[i][j].body);

			//Setting the position of the link
			leg[i][j].cX = x*altX;
			leg[i][j].cY = y*altY;
			leg[i][j].cZ = height - leg[i][j].totalLength/2.0 - (j > 0? leg[i][j-1].totalLength *j : 0);
			
			dBodySetPosition(leg[i][j].body, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ);			
		}

		if (i == 1)
			altX += 2;

		altY += 2;
		if (altY > 1)
			altY = -1;
	}

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
	//Attach the link joints
	for(int i = 0; i < LEG_NUM; ++i)
	{
		for(int j = 0; j < JOINT_NUM; ++j)
		{
			switch(j)
			{
				case 0: //Create a ball-in-socket joint and connect to the torso
					leg[i][j].joint = dJointCreateBall(world, 0);
					dJointAttach(leg[i][j].joint, leg[i][j].body, torso.body);
					dJointSetBallAnchor(leg[i][j].joint, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ + leg[i][j].totalLength/2.0);

					//For a ball-in-socket joint create a AMotor to control it
					leg[i][j].aMotor = dJointCreateAMotor(world, 0);
					dJointAttach(leg[i][j].aMotor, leg[i][j].body, torso.body);
					dJointSetAMotorMode(leg[i][j].aMotor, dAMotorUser);
					dJointSetAMotorNumAxes(leg[i][j].aMotor, 3);
					dJointSetAMotorAxis(leg[i][j].aMotor, 0, 1,  0, 0, 1);
					dJointSetAMotorAxis(leg[i][j].aMotor, 1, 1,  1, 0, 0);
					dJointSetAMotorAxis(leg[i][j].aMotor, 2, 1,  0, 1, 0);

					dJointSetAMotorParam(leg[i][j].aMotor, dParamLoStop,  -stop);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamHiStop,   stop);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamVel,      0);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamFMax,     fmax);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamBounce,   0);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamStopCFM,  cfm);
					dJointSetAMotorParam(leg[i][j].aMotor, dParamStopERP,  erp);

					//Hinge joint for test
					/*leg[i][j].joint = dJointCreateHinge(world, 0);
					dJointAttach(leg[i][j].joint, leg[i][j].body, torso.body);
					dJointSetHingeAnchor(leg[i][j].joint, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ + leg[i][j].totalLength/2.0);
					dJointSetHingeAxis(leg[i][j].joint, 0, 1, 0);*/
					break;
				case 1: //Create a hinge joint and connect to the previous link
					leg[i][j].joint = dJointCreateHinge(world, 0);
					dJointAttach(leg[i][j].joint, leg[i][j].body, leg[i][j-1].body);
					dJointSetHingeAnchor(leg[i][j].joint, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ + leg[i][j].totalLength/2.0);
					dJointSetHingeAxis(leg[i][j].joint, 0, 1, 0);
					break;
				case 2: //Create a universal joint and connect to the previous link
					leg[i][j].joint = dJointCreateUniversal(world, 0);
					dJointAttach(leg[i][j].joint, leg[i][j].body, leg[i][j-1].body);
					dJointSetUniversalAnchor(leg[i][j].joint, leg[i][j].cX, leg[i][j].cY, leg[i][j].cZ + leg[i][j].totalLength/2.0);
					dJointSetUniversalAxis1(leg[i][j].joint, 0, 1, 0);
					dJointSetUniversalAxis2(leg[i][j].joint, 0, 0, 1);
					break;
				default:
					break;
			}

			//Create the joint Controller
			leg[i][j].PD = new Controller(50, 5);
		}

	}

	skeleton = new AbstractSkeleton();
	LegFrame* shoulder = new LegFrame();
	LegFrame* hip = new LegFrame();
	for(int i = 0; i < 2; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			hip->leg[i][j] = &leg[i][j];
		}
	}

	for(int i = 2; i < LEG_NUM; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			shoulder->leg[i-2][j] = &leg[i][j];
		}
	}

	skeleton->setLegFrames(shoulder, hip);

}

void drawSkeleton ()
{
	//draw the torso
	dsSetColor(1.3, 1.3, 1.3);
	dsDrawCapsule(dBodyGetPosition(torso.body), dBodyGetRotation(torso.body), torso.length, torso.radius);

	//draw links
	LegFrame* drawFrame;
	for(int legFrame = 0; legFrame < 2; ++legFrame)
	{
		drawFrame = skeleton->getLegFrame(legFrame);
		for(int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < LINK_NUM; ++j)
			{
				if(j == LINK_NUM-1)
				{
					dReal dim[3] = {(*drawFrame->leg[i][j]).radius*2, (*drawFrame->leg[i][j]).radius*2, (*drawFrame->leg[i][j]).length};
					dsDrawBox(dBodyGetPosition((*drawFrame->leg[i][j]).body), dBodyGetRotation((*drawFrame->leg[i][j]).body), dim);
				}
				else
					dsDrawCapsule(dBodyGetPosition((*drawFrame->leg[i][j]).body), dBodyGetRotation((*drawFrame->leg[i][j]).body), (*drawFrame->leg[i][j]).length, (*drawFrame->leg[i][j]).radius);
			}
		}
	}

	dsDrawCapsule(dBodyGetPosition(head.body), dBodyGetRotation(head.body), head.length, head.radius);
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
	if(step < 3)
	{
	//Test
		LegFrame* legframe;
		for(int x = 0; x < 2; ++x)
		{
			legframe = skeleton->getLegFrame(x);
			for(int i = 0; i < 2; ++i)
			{
				for(int j = 0; j < JOINT_NUM; ++j)
				{
					/*glm::vec3 point = trajectories->getPointOnCurve(i, x);
					dReal pos[3] = {point.x, point.y, point.z};
		
					dsDrawSphereD(pos, dBodyGetRotation(leg[i][0].body), 0.01);*/
					float angle = 0;
					legframe->leg[i][j]->tX = 0;
					legframe->leg[i][j]->tZ = 0;
					switch(j)
					{
						case 0:
							if( x == 1 && i == 1)
							{
								if(step == 0)
									legframe->leg[i][j]->tY = -60 * M_PI/180.0f;
								else if(step == 1)
									legframe->leg[i][j]->tY = 15 * M_PI/180.0f;
								else if(step == 2)
									legframe->leg[i][j]->tY = -100 * M_PI/180.0f;
								legframe->legMode[i] = false; 
							}else if(x == 0 && i == 0)
							{
								if(step == 0)
									break;
								if(step == 1)
									legframe->leg[i][j]->tY = 95 * M_PI/180.0f;
								else if(step == 2)
									legframe->leg[i][j]->tY = -15 * M_PI/180.0f;
								legframe->legMode[i] = false; 
							}else
							{
								legframe->leg[i][j]->tX = 0;
								legframe->leg[i][j]->tY = 0;
								legframe->leg[i][j]->tZ = 0;

								legframe->legMode[i] = true;
							}
							break;
						case 1:
							if( x == 1 && i == 1)
							{
								if(step == 0)
									legframe->leg[i][j]->tY = 230 * M_PI/180.0f;
								else if(step == 1)
									legframe->leg[i][j]->tY = 140 * M_PI/180.0f;
								else if(step == 2)
									legframe->leg[i][j]->tY = 10 * M_PI/180.0f;
								legframe->legMode[i] = false; 
							}else if(x == 0 && i == 0)
							{
								if(step == 0)
									break;
								if(step == 1)
									legframe->leg[i][j]->tY = -120 * M_PI/180.0f;
								else if(step == 2)
									legframe->leg[i][j]->tY = -25 * M_PI/180.0f;

								legframe->legMode[i] = false; 
							}else
							{
								legframe->leg[i][j]->tX = 0;
								legframe->leg[i][j]->tY = 0;
								legframe->leg[i][j]->tZ = 0;

								legframe->legMode[i] = true;
							}
							break;
						case 2:
							break;
						default:
							break;
					}


					//legframe->leg[i][j]->tX = 0;
					//legframe->leg[i][j]->tY = //trajectories->getLegPitch(i, phase);
					//legframe->leg[i][j]->tZ = 0;

					//if(legframe->leg[i][j]->tY != 0)
					//	legframe->legMode[0] = true;

					//printf("%i angle is %f\n", step, legframe->leg[i][j]->tY);
				}
			}
		}
		++step;
	}
	//For standing
	//dReal aX = 0;
	//dReal aY = 0;
	//dReal aZ = 0;
	//for(int i = 0; i < LEG_NUM; ++i)
	//{
	//	for(int j = 0; j < LINK_NUM; ++j)
	//	{
	//		if( j == 1)
	//		{
	//			if(i > 1) //Rear legs
	//			{
	//				aY = -0.2;
	//			}
	//			else
	//			{
	//				aY = 0.2;
	//			}
	//		}
	//		else if( j == 2 )
	//		{
	//			if(i > 1)
	//			{
	//				aY = 0.1;
	//			}
	//			else
	//			{
	//				aY = -0.1;
	//			}
	//		}
	//		else
	//			aY = 0;

	//		leg[i][j].tX = aX;
	//		leg[i][j].tY = aY;
	//		leg[i][j].tZ = aZ;
	//	}
	//}
}

void ComputePDTorques()
{
	dReal maxF = 100.0;
	LegFrame* legframe;
	for(int x = 0; x < 2; ++x)
	{
		legframe = skeleton->getLegFrame(x);
		for (int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < LINK_NUM; ++j)
			{
				glm::vec3 angles = glm::vec3(0, 0, 0);
				glm::vec3 omega = glm::vec3(0, 0, 0);
				glm::vec3 result = glm::vec3(0, 0, 0);
				glm::vec3 delta = glm::vec3(0, 0, 0);
				switch(j)
				{
					case 0: //ball-and-socket joint
						angles[2] = dJointGetAMotorAngle(legframe->leg[i][j]->aMotor, 0);
						angles[0] = dJointGetAMotorAngle(legframe->leg[i][j]->aMotor, 1);
						angles[1] = dJointGetAMotorAngle(legframe->leg[i][j]->aMotor, 2);

						omega[2] = dJointGetAMotorAngleRate(legframe->leg[i][j]->aMotor, 0);
						omega[0] = dJointGetAMotorAngleRate(legframe->leg[i][j]->aMotor, 1);
						omega[1] = dJointGetAMotorAngleRate(legframe->leg[i][j]->aMotor, 2);

						delta = glm::vec3(legframe->leg[i][j]->tX - angles[0], legframe->leg[i][j]->tY - angles[1], legframe->leg[i][j]->tZ - angles[2]);
						result = legframe->leg[i][j]->PD->calculateControllerOutput(delta, omega);
						//printf("motor torques %d %d %d\n", result.x, result.y, result.z);
						legFrameTorques[x][i][j][0] = result.z;
						legFrameTorques[x][i][j][1] = result.x;
						legFrameTorques[x][i][j][2] = result.y;
						//dJointAddAMotorTorques(legframe->leg[i][j]->aMotor, result.z, result.x, result.y);
					
						break;
					case 1: //hinge joint
						angles[1] = dJointGetHingeAngle(legframe->leg[i][j]->joint);
						omega[1] = dJointGetHingeAngleRate(legframe->leg[i][j]->joint);
						omega[2] = 0;
						delta = glm::vec3(legframe->leg[i][j]->tX - angles[0], legframe->leg[i][j]->tY - angles[1], legframe->leg[i][j]->tZ - angles[2]);
						//printf("omega %d %d %d\n", omega[0], omega[1], omega[2]);
						result = legframe->leg[i][j]->PD->calculateControllerOutput(delta, omega);
						legFrameTorques[x][i][j][0] = 0;
						legFrameTorques[x][i][j][1] = result.y;
						legFrameTorques[x][i][j][2] = 0;
						//printf("result %d %d %d\n", result.x, result.y, result.z);
						//dJointAddHingeTorque(legframe->leg[i][j]->joint, result[1]);
						//dJointSetHingeParam(legframe->leg[i][j]->joint, dParamFMax, maxF);
						break;
					case 2: //universal joint
						angles[1] = dJointGetUniversalAngle1(legframe->leg[i][j]->joint);
						angles[2] = dJointGetUniversalAngle2(legframe->leg[i][j]->joint);
						omega[1] = dJointGetUniversalAngle1Rate(legframe->leg[i][j]->joint);
						omega[2] = dJointGetUniversalAngle2Rate(legframe->leg[i][j]->joint);
						delta = glm::vec3(legframe->leg[i][j]->tX - angles[0], legframe->leg[i][j]->tY - angles[1], legframe->leg[i][j]->tZ - angles[2]);
						result = legframe->leg[i][j]->PD->calculateControllerOutput(delta, omega);
						legFrameTorques[x][i][j][0] = 0;
						legFrameTorques[x][i][j][1] = result.y;
						legFrameTorques[x][i][j][2] = result.z;
						//dJointAddUniversalTorques(legframe->leg[i][j]->joint, result[1], 0);
						//dJointSetUniversalParam(legframe->leg[i][j]->joint, dParamFMax, maxF);
						//dJointSetUniversalParam(legframe->leg[i][j]->joint, dParamFMax1, maxF);
						break;
				}
			}
		}
	}
}

void ApplyLegFrameTorques()
{
	LegFrame* legframe;
	for(int x = 0; x < 2; ++x)
	{
		legframe = skeleton->getLegFrame(x);
		for(int i = 0; i < 2; ++i)
		{
			for(int j = 0; j < JOINT_NUM; ++j)
			{
				switch(j)
				{
					case 0:
						if(!legframe->legMode[i])
							dJointAddAMotorTorques(legframe->leg[i][j]->aMotor, legFrameTorques[x][i][j][0], legFrameTorques[x][i][j][1], legFrameTorques[x][i][j][2]);
						else
							if(i == 0)
								dJointAddAMotorTorques(legframe->leg[i][j]->aMotor, -0.5f*legFrameTorques[x][1][j][0], -0.5f*legFrameTorques[x][1][j][1], -0.5f*legFrameTorques[x][1][j][2]);
							else
								dJointAddAMotorTorques(legframe->leg[i][j]->aMotor, -0.5f*legFrameTorques[x][0][j][0], -0.5f*legFrameTorques[x][0][j][1], -0.5f*legFrameTorques[x][0][j][2]);
						break;
					case 1:
						dJointAddHingeTorque(legframe->leg[i][j]->joint, legFrameTorques[x][i][j][1]);
						break;
					case 2:
						dJointAddUniversalTorques(legframe->leg[i][j]->joint, legFrameTorques[x][i][j][1], legFrameTorques[x][i][j][2]);
						break;
					default:
						break;
				}

			}
		}
	}
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




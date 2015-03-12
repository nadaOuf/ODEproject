#include <ode\ode.h>
#include <drawstuff\drawstuff.h>
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;
dJointID joint;
static int flag = 0;
dsFunctions fn;


typedef struct {
	dBodyID body;
	dGeomID geom;
	dReal radius, length, mass;
} myLink;

myLink ball, pole;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	const int N = 10;
	dContact contact[N];

	int isGround = ((ground == o1) || (ground == o2));

	int n = dCollide(o1, o2, N, &contact[0].geom,sizeof(dContact));
	if(isGround)
	{
		if(n == 0)
			flag = 0;
		else
			flag = 1;

		for (int i = 0; i < n; ++i)
		{
			contact[i].surface.mode = dContactBounce;
			contact[i].surface.mu = dInfinity;
			contact[i].surface.bounce = 0.9999;
			contact[i].surface.bounce_vel = 0.1;
			dJointID c = dJointCreateContact (world, contactgroup, &contact[i]);
			dJointAttach (c, dGeomGetBody (contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}

}

static void start()
{
	static float xyz[3] = {0.0, -3.0, 1.0};
	static float hpr[3] = {90.0, 0.0, 0.0};
	dsSetViewpoint (xyz, hpr);
}

static void simLoop (int pause)
{
	const dReal *pos1, *R1, *pos2, *R2;
	dSpaceCollide (space, 0, &nearCallback);
	dWorldStep (world, 0.01);
	dJointGroupEmpty (contactgroup);

	dsSetColor(1.0, 0.0, 0.0);
	pos1 = dBodyGetPosition(ball.body);
	R1 = dBodyGetRotation(ball.body);
	dsDrawSphere(pos1, R1, ball.radius);

	pos2 = dBodyGetPosition(pole.body);
	R2 = dBodyGetRotation(pole.body);
	dsDrawCapsule(pos2, R2, pole.length, pole.radius);
}


void createBallPole()
{
	dMass m1, m2;
	dReal x0 = 0.0, y0 = 0.0, z0 = 2.5;

	//Creating the ball
	ball.radius = 0.2;
	ball.mass = 1.0;
	ball.body = dBodyCreate(world);
	dMassSetZero(&m1);
	dMassSetSphereTotal(&m1, ball.mass, ball.radius);
	dBodySetMass(ball.body, &m1);
	dBodySetPosition(ball.body, x0, y0, z0);
	ball.geom = dCreateSphere(space, ball.radius);
	dGeomSetBody (ball.geom, ball.body);

	//Creating the capsule
	pole.radius = 0.025;
	pole.length = 1.0;
	pole.mass = 1.0;
	pole.body = dBodyCreate(world);
	dMassSetZero(&m2);
	dMassSetCapsuleTotal(&m2, pole.mass, 3, pole.radius, pole.length);
	dBodySetMass(pole.body, &m2);
	dBodySetPosition(pole.body, x0, y0, z0 - 0.5 *pole.length);
	pole.geom = dCreateCapsule(space, pole.radius, pole.length);
	dGeomSetBody(pole.geom, pole.body);

	//Creating the hinge joint
	joint = dJointCreateHinge(world, 0);
	dJointAttach(joint, ball.body, pole.body);
	dJointSetHingeAnchor(joint, x0, y0, z0 - ball.radius);
	dJointSetHingeAxis(joint, 1, 0, 0);

}

void prepDrawStuff() 
{
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = 0;
	fn.stop = 0;
	fn.path_to_textures = "../Debug/textures";
}


int main(int argc, char **argv)
{
	prepDrawStuff();

	dInitODE();
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(world, 0, 0, -9.8);
	ground = dCreatePlane(space, 0, 0, 1, 0);
	createBallPole();

	dsSimulationLoop(argc, argv, 352, 288, &fn);

	dWorldDestroy(world);
	dCloseODE();
	return 0;

}
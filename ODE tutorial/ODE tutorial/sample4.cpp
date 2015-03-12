#include <ode\ode.h>
#include <drawstuff\drawstuff.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif

#ifdef dDOUBLE
#define dsDrawBox		dsDrawBoxD
#define dsDrawSphere	dsDrawSphereD
#define dsDrawCylinder	dsDrawCylinderD
#define dsDrawCapsule	dsDrawCapsuleD
#define dsDrawLine		dsDrawLineD
#endif

#define DENSITY (5.0)

struct MyObject {
	dBodyID body;
};

dReal radius = 0.25;
dReal length = 1.0;
dReal sides[3] = {0.5, 0.5, 1.0};
static dWorldID world;
static MyObject sphere, box, capsule, cylinder;

static void start()
{
	static float xyz[3] = {5, 3, 0.5};
	static float hpr[3] = {-180, 0, 0};
	dsSetViewpoint (xyz, hpr);
}

static void simLoop (int pause)
{
	const dReal *pos1, *R1, *pos2, *R2, *pos3, *R3;
	dsSetColor(1,0,0);
	dsSetSphereQuality(3);
	pos1= dBodyGetPosition(sphere.body);
	R1	= dBodyGetRotation(sphere.body);
	dsDrawSphere (pos1, R1, radius);

	dsSetColorAlpha (0,1,0,1);
	pos2= dBodyGetPosition(cylinder.body);
	R2	= dBodyGetRotation(cylinder.body);
	dsDrawCylinder(pos2, R2, length, radius);

	pos2= dBodyGetPosition(capsule.body);
	R2	= dBodyGetRotation(capsule.body);
	dsDrawCapsule(pos2, R2, length, radius);

	dsSetColorAlpha(0,0,1,1);
	pos3= dBodyGetPosition(box.body);
	R3	= dBodyGetRotation(box.body);
	dsDrawBox(pos3, R3, sides);
	dReal posA[3] = {0, 5, 0}, posB[3] = {0, 5, 1.9};
	dsDrawLine(posA, posB);
}

int main (int argc, char **argv)
{
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = 0;
	fn.stop = 0;
	fn.path_to_textures = "..\\Debug\\textures";

	dInitODE();
	world = dWorldCreate();

	dMass m;
	dMassSetZero (&m);

	sphere.body = dBodyCreate (world);
	dReal radius = 0.5;
	dMassSetSphere(&m, DENSITY, radius);
	dBodySetMass (sphere.body, &m);
	dBodySetPosition (sphere.body, 0, 1, 1);

	box.body = dBodyCreate (world);
	dMassSetBox (&m, DENSITY, sides[0], sides[1], sides[2]);
	dBodySetMass (box.body, &m);
	dBodySetPosition (box.body, 0, 2, 1);

	capsule.body = dBodyCreate (world);
	dMassSetCapsule (&m, DENSITY, 3, radius, length);
	dBodySetMass (capsule.body, &m);
	dBodySetPosition (capsule.body, 0, 4, 1);

	cylinder.body = dBodyCreate (world);
	dMassSetCylinder (&m, DENSITY, 3, radius, length);
	dBodySetMass (cylinder.body, &m);
	dBodySetPosition (cylinder.body, 0, 3, 1);

	dsSimulationLoop(argc, argv, 960, 480, &fn);
	dWorldDestroy(world);
	dCloseODE();
	return 0;
}
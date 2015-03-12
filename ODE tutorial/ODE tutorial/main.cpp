#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef  dDOUBLE
#define dsDrawSphere dsDrawSphereD
#endif

// dynamics and collision objects
//World and space to put everything in
static dWorldID world;
static dSpaceID space;

//A body with associated geometry and mass
static dBodyID body;	
static dGeomID geom;	
static dMass m;

//A joint group to store the contact joints that are created during a collision
static dJointGroupID contactgroup;

//This is called by dSpaceCollide when two objects in space are potentailly in collisding
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	//retrieving the geometry body
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	//creating contact
	dContact contact;
	//Checking for the contact mode
	contact.surface.mode = dContactBounce | dContactSoftCFM;
	//set the friction parameter
	contact.surface.mu = dInfinity;
	//set the amount of "bouncyness"
	contact.surface.bounce = 0.9;
	//set the minimum velocity needed to cause a bounce
	contact.surface.bounce_vel = 0.1;
	//set the constraint force mixing parameter
	contact.surface.soft_cfm = 0.001;
	if(int numc = dCollide (o1, o2, 1, &contact.geom, sizeof(dContact))) {
		dJointID c = dJointCreateContact (world, contactgroup, &contact);
		dJointAttach (c, b1, b2);
	}

}

//Start Simulation function
//set the viewpoint of the camera
static void start()
{
	static float xyz[3] = {2.0f, -2.0f, 1.7600f};
	static float hpr[3] = {140.000f, -17.0000f, 0.00000f};
	dsSetViewpoint (xyz, hpr);
}

// simulation loop
static void simLoop (int pause)
{
	const dReal *pos;
	const dReal *R;
	//Find collisions and add contact joints
	dSpaceCollide (space, 0, &nearCallback);

	//Increment the simulation
	dWorldQuickStep (world, 0.01);

	//Remove all contact joints
	dJointGroupEmpty (contactgroup);

	//Redraw sphere at new location
	pos = dGeomGetPosition (geom);
	R = dGeomGetRotation (geom);
	dsDrawSphere (pos, R, dGeomSphereGetRadius (geom));
}

int main(int argc, char **argv)
{
	// Setup pointer to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.command = 0;
	fn.path_to_textures = "..\\Debug\\textures";

	dInitODE();

	//create world
	world = dWorldCreate ();
	space = dHashSpaceCreate (0);
	dWorldSetGravity (world, 0, 0, -0.2);
	dWorldSetCFM (world, 1e-5);
	dCreatePlane (space, 0, 0, 1, 0);
	contactgroup = dJointGroupCreate (0);

	//create object
	body = dBodyCreate (world);
	geom = dCreateSphere (space, 0.5);
	dMassSetSphere (&m, 1, 0.5);
	dBodySetMass (body, &m);
	dGeomSetBody (geom, body);

	//set initial position
	dBodySetPosition (body, 0, 0, 3);

	//run simulation
	dsSimulationLoop (argc, argv, 352, 288, &fn);

	//clean up
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
	return 0;

}

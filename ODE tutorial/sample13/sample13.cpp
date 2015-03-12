//  sample13.cpp  3 DOF manipulator by Kosei Demura  2006-2011
// Change logs
// 2009-03-12: Addition of English comments, and change initialize of
//             mass parameter mass in the main funciton; Thanks Richard Kooijman.
#include <ode/ode.h>             // ODE
#include <drawstuff/drawstuff.h> // The drawing library for ODE; 
#define NUM 4         // No. of links;

dWorldID    world;         // a dynamic world; 
dBodyID     link[NUM];     // links, link[0] is a base; [0]
dJointID    joint[NUM];    // joints, joint[0] is a fixed joint; 関節    joint[0]
static double THETA[NUM] = { 0.0, 0.0, 0.0, 0.0}; // target angle of joints; [rad]
static double l[NUM]  = { 0.10, 0.90, 1.00, 1.00};  // length of links; [m]
static double r[NUM]  = { 0.20, 0.04, 0.04, 0.04};  // radius of links; [m]

void control()    /***  p control;   ****/
{
    static int step = 0;     // step number; 
    double k1 =  10.0,  fMax  = 100.0; // k1:gain, fMax: max torqeu; k1:,  [Nm]
    printf("\r%6d:",step++);
    for (int j = 1; j <NUM; j++)
    {
        double tmpAngle = dJointGetHingeAngle(joint[j]);  // current joint angle; [rad]
        double z = THETA[j] - tmpAngle;  // z= target - current; z: =
        dJointSetHingeParam(joint[j],  dParamVel,  k1*z); // angular velocity; 
        dJointSetHingeParam(joint[j], dParamFMax, fMax); // max torqeu; 
    }
}

void start()   /*** Initialize drawing API;  ***/
{
    float xyz[3] = {  3.04, 1.28, 0.76};   // view point; x, y, z　[m]
    float hpr[3] = { -160.0, 4.50, 0.00};  // view direction; (heading, pitch, roll)　[°]
    dsSetViewpoint(xyz,hpr);               // setting of view point and direction; 
}

void command(int cmd)   /***  key control function;  ***/
{
    switch (cmd)
    {
    case 'j':
        THETA[1] += 0.05; //[1].05[rad]
        // increases THETA[1] when j key is pressed
        break;
    case 'f':
        THETA[1] -= 0.05;
        break;
    case 'k':
        THETA[2] += 0.05;
        break;
    case 'd':
        THETA[2] -= 0.05;
        break;
    case 'l':
        THETA[3] += 0.05;
        break;
    case 's':
        THETA[3] -= 0.05;
        break;
    }

    // English: limit target angles not to destroy a robot
    // Japanese:
    if (THETA[1] <  - M_PI)    THETA[1] =  - M_PI; // 
    if (THETA[1] >    M_PI)    THETA[1] =    M_PI;
    if (THETA[2] < -2*M_PI/3)  THETA[2] =  - 2*M_PI/3;
    if (THETA[2] >  2*M_PI/3)  THETA[2] =    2*M_PI/3;
    if (THETA[3] < -2*M_PI/3)  THETA[3] =  - 2*M_PI/3;
    if (THETA[3] >  2*M_PI/3)  THETA[3] =    2*M_PI/3;
}

// English: simulation loop.
// Japanese: 
void simLoop(int pause)
{
    control();
    dWorldStep(world, 0.02);
    // draw a robot; 
    dsSetColor(1.0,1.0,1.0); // set color; (r, g, b) 
    for (int i = 0; i <NUM; i++ ) // 
        dsDrawCapsuleD(dBodyGetPosition(link[i]), dBodyGetRotation(link[i]), l[i], r[i]);
}

int main(int argc, char *argv[])
{
    dsFunctions fn; // an variable for drawstuff; 

    double x[NUM] = {0.00}, y[NUM] = {0.00};  // COG; 
    double z[NUM]         = { 0.05, 0.50, 1.50, 2.55};
    double m[NUM] = {10.00, 2.00, 2.00, 2.00};           // mass; 
    double anchor_x[NUM]  = {0.00}, anchor_y[NUM] = {0.00}; // anchor point; 
    double anchor_z[NUM] = { 0.00, 0.10, 1.00, 2.00};
    double axis_x[NUM]  = { 0.00, 0.00, 0.00, 0.00};  // rotation axis; 
    double axis_y[NUM]  = { 0.00, 0.00, 1.00, 1.00};
    double axis_z[NUM]  = { 1.00, 1.00, 0.00, 0.00};

    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.path_to_textures = "../Debug/textures";

    dInitODE();  // Initalize ODE; 
    world = dWorldCreate();  // create a dynamic world; 
    dWorldSetGravity(world, 0, 0, -9.8); // set gravity; 

    for (int i = 0; i <NUM; i++)         // creation and setting of links; 
    {
        dMass mass;  // mass parameter; 
        link[i] = dBodyCreate(world);     // create link; 
        dBodySetPosition(link[i], x[i], y[i], z[i]); // set position; 
        dMassSetZero(&mass);      // initialize the mass parameter; 
        dMassSetCapsuleTotal(&mass,m[i],3,r[i],l[i]);  // calculate the mass parameter; 
        dBodySetMass(link[i], &mass);  // set the mass parameter to the body; 
    }

    joint[0] = dJointCreateFixed(world, 0); // a fixed joint;
    dJointAttach(joint[0], link[0], 0);     // attach the fixed joint to the ground;
    dJointSetFixed(joint[0]);               // set the fixed joint;
    for (int j = 1; j <NUM; j++)
    {
        joint[j] = dJointCreateHinge(world, 0);     // create a hinge joint;
        dJointAttach(joint[j], link[j-1], link[j]); // attach the joints;
        dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]); // set an anchor point;
        dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]); // set an rotation axis;
    }
    dsSimulationLoop(argc, argv, 640, 570, &fn); // simulation loop;
    dCloseODE(); // close ODE
    return 0;
}

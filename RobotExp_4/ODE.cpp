#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define MAX_JOINT_NUM 2		// maximum joint num

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;
static dGeomID g_Ground;	//ground address


Object g_oObj[MAX_JOINT_NUM + 1];	// link global parameter
static dJointID g_oJoint[MAX_JOINT_NUM + 1]; // joint global parameter

double g_tar_q[MAX_JOINT_NUM] = { 0.0, 0.0 };	// joint trarget position
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0 };	// current joint position

void InitDrawStuff() {

	g_Fn.version = DS_VERSION;
	g_Fn.start = &StartDrawStuff;
	g_Fn.step = &SimLoopDrawStuff;
	g_Fn.command = &CommandDrawStuff;
	g_Fn.stop = StopDrawStuff;
	g_Fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
}


void InitODE() {
	// ode main function
	// initiating world options
	dInitODE();
	g_World = dWorldCreate();
	g_Space = dHashSpaceCreate(0);
	g_Contactgroup = dJointGroupCreate(0);
	g_Ground = dCreatePlane(g_Space, 0, 0, 1, 0);

	dWorldSetGravity(g_World, 0, 0, -9.8);	// gravity setting X,Y,Z
	dWorldSetERP(g_World, 0.2);
	dWorldSetCFM(g_World, 1e-5);

}



void RunODE(size_t width, size_t height) {
	//page 5
	//TO DO
	InitDrawStuff();
	InitODE();

	InitRobot();

	dsSimulationLoop(0, 0, width, height, &g_Fn);
}



void ReleaseODE() {

	dJointGroupDestroy(g_Contactgroup);
	dSpaceDestroy(g_Space);
	dWorldDestroy(g_World);
	dCloseODE();
}



void StartDrawStuff() {

	//TO DO

}


void SimLoopDrawStuff(int pause)
{
	//page 6
	//TO DO
	PControl();

	double dt = 2.1;
	dWorldStep(g_World, dt);

	dReal r, length;
	dVector3 length_box;

	//draw capsule object
	dsSetColor(0.2, 0.2, 0.7);
	dGeomCapsuleGetParams(g_oObj[0].geom, &r, &length);	//set capsule params load from g_oObj
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[0].body), dBodyGetRotation(g_oObj[0].body), (float)length, (float)r );	//load geom from init robot function
	
	dGeomCapsuleGetParams(g_oObj[1].geom, &r, &length);	//set capsule params load from g_oObj
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[1].body), dBodyGetRotation(g_oObj[1].body), (float)length, (float)r);
}



void CommandDrawStuff(int cmd) {

	//TO DO

}



void StopDrawStuff() {

	//TO DO

}


void InitRobot()
{
	//body
	dMass mass;
	dMatrix3 R;

	//mass point
	dReal x[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal y[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal z[MAX_JOINT_NUM] = { 0.5, 1.25 };

	//link pose
	dReal ori_x[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal ori_y[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal ori_z[MAX_JOINT_NUM] = { 1.0, 1.0 };
	dReal ori_q[MAX_JOINT_NUM] = { 0.0, 0.0 };

	//link length
	dReal length[MAX_JOINT_NUM] = { 1.0, 0.5 };

	//mass of A
	dReal weight[MAX_JOINT_NUM] = { 1.0,1.0 };

	dReal r[MAX_JOINT_NUM];
	for (int i = 0; i < MAX_JOINT_NUM; i++)
	{
		r[i] = 0.125;
	}

	//creating body
	for (int i = 0; i < MAX_JOINT_NUM; i++)
	{
		// Make Body gunctions
		g_oObj[i].body = dBodyCreate(g_World);	// creating body and return address of body
		dBodySetPosition(g_oObj[i].body, x[i], y[i], z[i]);	//move body to position x,y,z (body address, x, y, z)
		dMassSetZero(&mass);	// set body mass zero (initalize)
		dMassSetCapsuleTotal(&mass, weight[i], 1, r[i], length[i]);	//capsule mass setting (mass, heading, radious, length)
		dBodySetMass(g_oObj[i].body, &mass);	// set body mass
		g_oObj[i].geom = dCreateCapsule(g_Space, r[i], length[i]);	// save gobj (capsule)
		dGeomSetBody(g_oObj[i].geom, g_oObj[i].body);	//creating geometry fit to geom option (capsule)
		dRFromAxisAndAngle(R, ori_x[i], ori_y[i], ori_z[i], ori_q[i]);	// body angular setting
		dBodySetRotation(g_oObj[i].body, R);	// change body with angular matrix
	}

	//Joints
	dReal c_x[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal c_y[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal c_z[MAX_JOINT_NUM] = { 0.0, 1.0 };

	dReal axis_x[MAX_JOINT_NUM] = { 0.0, 0.0 };
	dReal axis_y[MAX_JOINT_NUM] = { 0.0, 1.0 };
	dReal axis_z[MAX_JOINT_NUM] = { 1.0, 0.0 };

	// fixed axis setting
	g_oJoint[0] = dJointCreateFixed(g_World, 0);	//set gobj[0] fixed joint
	dJointAttach(g_oJoint[0], 0, g_oObj[0].body);	// if second arg is 0, attach with world
	dJointSetFixed(g_oJoint[0]);


	// joint 1 setting : can converted after with adding joint
	for (int i = 1; i < MAX_JOINT_NUM; i++)	// (joint 0 setted before)
	{
		/* code */
		g_oJoint[i] = dJointCreateHinge(g_World, 0);
		dJointAttach(g_oJoint[i], g_oObj[i].body, g_oObj[i - 1].body);	// attach joint 0 and joint 1 (obj0 and obj1)
		dJointSetHingeAnchor(g_oJoint[i], c_x[i], c_y[i], c_z[i]);	// attach objects with hindge joint
		dJointSetHingeAxis(g_oJoint[i], axis_x[i], axis_y[i], axis_z[i]);
	}


}

void PControl()	//hindge joint Pcontrol
{

	//TO DO

}
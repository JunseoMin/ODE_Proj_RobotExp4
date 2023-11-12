#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define MAX_JOINT_NUM 3		// maximum joint num

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;
static dGeomID g_Ground;	//ground address


Object g_oObj[MAX_JOINT_NUM + 1];	// link global parameter
static dJointID g_oJoint[MAX_JOINT_NUM + 1]; // joint global parameter

double g_tar_q[MAX_JOINT_NUM] = { 0.0 , 45. * DEG2RAD, 30. * DEG2RAD };	// joint trarget position
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };	// current joint position

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

	dWorldSetGravity(g_World, 0, 0, -GRAVITY);	// gravity setting X,Y,Z
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
	float dPose[MAX_JOINT_NUM] = { 2.0, 2.0, 3.0 };
	float dRot[MAX_JOINT_NUM] = { 0.3, 90 * DEG2RAD, 90*DEG2RAD };
	dsSetViewpoint(dPose, dRot);

}

void nearCB(void* data, dGeomID o1, dGeomID o2) {
	int i;
	static const int N = 5;
	dContact contact[N];

	if ((g_Ground == o1) || (g_Ground == o2))	// if object collide with ground
	{
		bool isGround = ((g_Ground == o1) || (g_Ground == o2));
		int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

		if (isGround)
		{
			for (int i = 0; i < n; i++)
			{
				contact[i].surface.mu = 0.1;
				contact[i].surface.mode = dContactBounce;
				contact[i].surface.bounce = 0.0;
				contact[i].surface.bounce_vel = 0.0;

				dJointID c = dJointCreateContact(g_World, g_Contactgroup, &contact[i]);

				dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
			}
		}
	}
	else
	{
		//obstacle collide between object codes need collision box
	}
}


void SimLoopDrawStuff(int pause)
{
	//page 6
	//TO DO
	//dSpaceCollide(g_Space, 0, &nearCB);
	
	PControl();

	double dt = 0.1;
	dWorldStep(g_World, dt);

	dReal r, length;
	dVector3 length_box;

	//draw capsule object

	/* for (int i = 0; i < MAX_JOINT_NUM; i++)
	{
		dsSetColor(0.5 - i / 20, 0.2+i/10, 0.7-i/10);
		dGeomCapsuleGetParams(g_oObj[i].geom, &r, &length);	//set capsule params load from g_oObj
		dsDrawCapsuleD(dBodyGetPosition(g_oObj[i].body), dBodyGetRotation(g_oObj[i].body), (float)length, (float)r);	//load geom from init robot function : type casted
	}
	*/

	dsSetColor(0., 0.4, 0.4);
	dGeomCapsuleGetParams(g_oObj[0].geom, &r, &length);	//set capsule params load from g_oObj
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[0].body), dBodyGetRotation(g_oObj[0].body), (float)length, (float)r);	//load geom from init robot function : type casted

	dsSetColor(0.5, 0.0, 0.0);
	dGeomCapsuleGetParams(g_oObj[1].geom, &r, &length);	//set capsule params load from g_oObj
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[1].body), dBodyGetRotation(g_oObj[1].body), (float)length, (float)r);	//load geom from init robot function : type casted

	dsSetColor(0.5, 0., 0.7);
	dGeomCapsuleGetParams(g_oObj[2].geom, &r, &length);	//set capsule params load from g_oObj
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[2].body), dBodyGetRotation(g_oObj[2].body), (float)length, (float)r);	//load geom from init robot function : type casted
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
	dReal x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal z[MAX_JOINT_NUM] = { 0.25, 1.0, 1.75 };

	//link pose
	dReal ori_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_z[MAX_JOINT_NUM] = { 1.0, 1.0, 1.0 };
	dReal ori_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };

	//link length
	dReal length[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

	//mass of A
	dReal weight[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

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

	//center of hindge
	dReal c_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_z[MAX_JOINT_NUM] = { 0.0, 0.5, 1.5 };

	//hindge axis
	dReal axis_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal axis_y[MAX_JOINT_NUM] = { 0.0, 1.0, 1.0 };
	dReal axis_z[MAX_JOINT_NUM] = { 1.0, 0.0, 0.0 };

	// fixed axis setting
	g_oJoint[0] = dJointCreateFixed(g_World, 0);	//set gobj[0] fixed joint
	dJointAttach(g_oJoint[0], 0, g_oObj[0].body);	// if second arg is 0, attach with world
	dJointSetFixed(g_oJoint[0]);


	// joint settings : can converted after with adding joint
	for (int i = 1; i < MAX_JOINT_NUM; i++)	// (joint 0 setted before)
	{
		g_oJoint[i] = dJointCreateHinge(g_World, 0);
		dJointAttach(g_oJoint[i], g_oObj[i].body, g_oObj[i - 1].body);	// attach joint 0 and joint 1 (obj0 and obj1)
		dJointSetHingeAnchor(g_oJoint[i], c_x[i], c_y[i], c_z[i]);	// attach objects with hindge joint
		dJointSetHingeAxis(g_oJoint[i], axis_x[i], axis_y[i], axis_z[i]);
	}


}

void PControl()	//hindge joint Pcontrol
{
	dReal Kp = 10, fMax = 5000.0;
	dReal a_error_q[MAX_JOINT_NUM];

	for (int i = 1; i < MAX_JOINT_NUM; i++)
	{
		g_cur_q[i] = dJointGetHingeAngle(g_oJoint[i]);

		a_error_q[i] = g_tar_q[i] - g_cur_q[i];
		//gymbol lock control
		if (g_tar_q[i] - g_cur_q[i] > 180.0*DEG2RAD)
		{
			g_cur_q[i] += 359.9*DEG2RAD;
		}

		if (g_tar_q[i] - g_cur_q[i] < -180.0*DEG2RAD)
		{
			g_cur_q[i] -= 359.9*DEG2RAD;
		}
		
		dJointSetHingeParam(g_oJoint[i], dParamVel, Kp*a_error_q[i]);
		dJointSetHingeParam(g_oJoint[i], dParamFMax, fMax);
	}
}

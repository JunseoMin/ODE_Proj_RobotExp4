#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define MAX_JOINT_NUM 2

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;

Object g_oObj[MAX_JOINT_NUM + 1];
static dJointID g_oJoint[MAX_JOINT_NUM + 1];

double g_tar_q[MAX_JOINT_NUM] = { 0.0, 0.0};
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0};

void InitDrawStuff() {

	g_Fn.version = DS_VERSION;
	g_Fn.start = &StartDrawStuff;
	g_Fn.step = &SimLoopDrawStuff;
	g_Fn.command = &CommandDrawStuff;
	g_Fn.stop = StopDrawStuff;
	g_Fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
}


void InitODE() {

	//TO DO
}



void RunODE(size_t width, size_t height) {

	//TO DO
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

	//TO DO
	
}



void CommandDrawStuff(int cmd) {

	//TO DO

}



void StopDrawStuff() {

	//TO DO

}


void InitRobot()
{

	//TO DO
	
}

void PControl()
{

	//TO DO

}
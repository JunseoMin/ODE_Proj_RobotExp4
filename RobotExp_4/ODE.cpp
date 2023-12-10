#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define PI 3.141592
#define MAX_JOINT_NUM 3

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;
dGeomID g_Ground;

Object g_oObj[MAX_JOINT_NUM + 1];
static dJointID g_oJoint[MAX_JOINT_NUM + 1];

double g_tar_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };

double _mod(double dividend, double divisor) {
	// 부호 상관 없이 모듈러 연산 수행
	return std::fmod((std::fmod(dividend, divisor) + divisor), divisor);
}

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
	dInitODE();
	g_World = dWorldCreate();
	g_Space = dHashSpaceCreate(0);
	g_Contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(g_World, 0, 0, -GRAVITY);
	g_Ground = dCreatePlane(g_Space, 0, 0, 1, 0);
	dWorldSetCFM(g_World, 1e-5);
	//dWorldSetERP(g_World, 1.0);

}

void RunODE(size_t width, size_t height) {

	//TO DO
	InitDrawStuff();
	InitODE();

	InitRobot();

	//simulation part
	dsSimulationLoop(0, 0, width, height, &g_Fn);

}

void ReleaseODE() {
	dJointGroupDestroy(g_Contactgroup);
	dSpaceDestroy(g_Space);
	dWorldDestroy(g_World);
	dCloseODE();

}

// 초기 위치 평가자료대로 설정 
void StartDrawStuff() {

	//TO DO
	float dPos[3] = { 0.0, 3.0, 2.0 };
	float dRot[3] = { -90.0, -20.0, 0.0 };
	dsSetViewpoint(dPos, dRot);

}

void SimLoopDrawStuff(int pause)
{
	//TO DO

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	g_tar_q[0] = jointData.Q_tar[0];
	g_tar_q[1] = jointData.Q_tar[1];

	jointData.Q_cur[0] = g_cur_q[0];
	jointData.Q_cur[1] = g_cur_q[1];

	jointData.Q_tar[0] = g_tar_q[0];
	jointData.Q_tar[1] = g_tar_q[1];

	SET_SYSTEM_MEMORY("JointData", jointData);

	//Robot Control
	PControl();

	dReal length, radius;

	//Close Loop Setting
	double dt = 0.1;
	dWorldStep(g_World, dt);

	// Draw Robot
	// 준서가 말한대로 색 지정
	dsSetColor(0., 0., 0.);
	dGeomCapsuleGetParams(g_oObj[0].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[0].body), dBodyGetRotation(g_oObj[0].body), (float)length, (float)radius);
	// 반지름 r, 길이 L인 Capsule Geometry를 Space 작성하고 그 ID번호를 반환.

	dsSetColor(0.5, 0.5, 0.5);
	dGeomCapsuleGetParams(g_oObj[1].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[1].body), dBodyGetRotation(g_oObj[1].body), (float)length, (float)radius);
	// 반지름 r, 길이 L인 Capsule Geometry를 Space 작성하고 그 ID번호를 반환.

	dsSetColor(1., 1., 1.);
	dGeomCapsuleGetParams(g_oObj[2].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[2].body), dBodyGetRotation(g_oObj[2].body), (float)length, (float)radius);
	// 반지름 r, 길이 L인 Capsule Geometry를 Space 작성하고 그 ID번호를 반환.

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

	dMass mass;
	dMatrix3 R;

	//캡슐의 반지름
	dReal r[MAX_JOINT_NUM];

	for (int i = 0; i < MAX_JOINT_NUM; i++)
		r[i] = 0.125;

	//캡슐(링크)의 길이
	// 텀 프로젝트 기준대로 축 길이 설정 
	dReal length[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

	//캡슐(링크)의 무게
	// 텀 프로젝트 기준대로 설정한 길이에 따라 무게 설정 
	dReal weight[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

	// Body 설정
	//질량 중심점
	// 텀 프로젝트 기준대로 질량 중심점 설정
	dReal x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal z[MAX_JOINT_NUM] = { 0.25, 1.0, 1.75 };

	//링크 pose
	// 텀 프로젝트 기준대로 설정한 링크 자세 설정
	dReal ori_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_z[MAX_JOINT_NUM] = { 1.0, 1.0, 1.0 };
	dReal ori_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };

	// Joint 설정 
	//각 조인트의 회전축의 위치
	dReal c_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_z[MAX_JOINT_NUM] = { 0.0, 0.5, 1.5 };

	//hindge axis
	// 텀 프로젝트에 맞춰 시계방향을 하도록 설정
	dReal axis_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal axis_y[MAX_JOINT_NUM] = { 0.0, -1.0, -1.0 };
	dReal axis_z[MAX_JOINT_NUM] = { 1.0, 0.0, 0.0 };


	for (int i = 0; i < MAX_JOINT_NUM; i++)
	{
		g_oObj[i].body = dBodyCreate(g_World);								// dBodyCreate 함수로 각 링크의 바디를 생성 -> body를 생성하고 id 반환한다.
		dBodySetPosition(g_oObj[i].body, x[i], y[i], z[i]);					// 바디의 초기 위치 설정 -> body를 해당 위치로 이동한다.
		dMassSetZero(&mass);												// 질량 구조체 초기화
		dMassSetCapsuleTotal(&mass, weight[i], 1, length[i], r[i]);			// dCreateCapsule 함수로 질량과 장축의 방향, 반지름, 길이 정보를 설정한다. -> 1은 z축과 일치함을 나타낸다. 
		dBodySetMass(g_oObj[i].body, &mass);								// 바디에 질량 정보 설정
		g_oObj[i].geom = dCreateCapsule(g_Space, length[i], r[i]);			// dCreateCapsule 함수로 반지름이 r이고, 길이가 l인 캡슐 모양을 space이 나타내고 이것의 id를 반환한다.
		dGeomSetBody(g_oObj[i].geom, g_oObj[i].body);						// 지오메트리에 바디 설정 -> body의 모양에 맞춰 geometry를 설정한다.
		dRFromAxisAndAngle(R, ori_x[i], ori_y[i], ori_z[i], ori_q[i]);		// 회전 설정 -> rotation matrix에 회전 축과 회전 각도를 입력으로 넣는다. 
		dBodySetRotation(g_oObj[i].body, R);								// 바디에 회전 매트릭스를 적용하여 나타낸다. 
	}


	// 고정 축 설정 
	g_oJoint[0] = dJointCreateFixed(g_World, 0);						// 고정 Joint 생성
	dJointAttach(g_oJoint[0], 0, g_oObj[0].body);						// body1과 body2를 결합, Body1 혹은 body2가 ‘0’이면 World와 결합
	dJointSetFixed(g_oJoint[0]);

	// Joint 설정 
	for (int i = 1; i < MAX_JOINT_NUM; i++)
	{
		g_oJoint[i] = dJointCreateHinge(g_World, 0);
		dJointAttach(g_oJoint[i], g_oObj[i].body, g_oObj[i - 1].body);
		dJointSetHingeAnchor(g_oJoint[i], c_x[i], c_y[i], c_z[i]);			// Hinge의 중심점( x, y, z )를 설정
		dJointSetHingeAxis(g_oJoint[i], axis_x[i], axis_y[i], axis_z[i]);	// Hinge의 회전축 벡터( x, y, z)를 설정
	}
	//g_oJoint[1] = PI/2;

}

void PControl()
{
	
	if (g_tar_q[0] >= 360)	 g_tar_q[0] -= 360;
	else if (g_tar_q[0] == 180) g_tar_q[0] = g_tar_q[0] - 0.01;
	else if (g_tar_q[0] > 180) g_tar_q[0] -= 360;

	if (g_tar_q[0] <= -360) g_tar_q[0] += 360;
	else if (g_tar_q[0] == -180) g_tar_q[0] = g_tar_q[0] + 0.01;
	else if (g_tar_q[0] < -180) g_tar_q[0] += 360;


	if (g_tar_q[1] >= 360)	g_tar_q[1] -= 360;
	else if (g_tar_q[1] == 180) g_tar_q[1] = g_tar_q[1] - 0.01;
	else if (g_tar_q[1] > 180) g_tar_q[1] -= 360;

	if (g_tar_q[1] <= -360) g_tar_q[1] += 360;
	else if (g_tar_q[1] == -180) g_tar_q[1] = g_tar_q[1] + 0.01;
	else if (g_tar_q[1] < -180) g_tar_q[1] += 360;

	double theta_cur1 = dJointGetHingeAngle(g_oJoint[1]);
	double theta_des1 = g_tar_q[0] * DEG2RAD;
	double theta_err1 = theta_des1 - theta_cur1;
	double velo_des1 = 0.1 * theta_err1;

	g_cur_q[0] = theta_cur1 * RAD2DEG;

	dJointSetHingeParam(g_oJoint[1], dParamVel, velo_des1);
	dJointSetHingeParam(g_oJoint[1], dParamFMax, 100);

	double theta_cur2 = dJointGetHingeAngle(g_oJoint[2]);
	double theta_des2 = g_tar_q[1] * DEG2RAD;
	double theta_err2 = theta_des2 - theta_cur2;
	double velo_des2 = 0.1 * theta_err2;

	g_cur_q[1] = theta_cur2 * RAD2DEG;

	dJointSetHingeParam(g_oJoint[2], dParamVel, velo_des2);
	dJointSetHingeParam(g_oJoint[2], dParamFMax, 100);
	/*
	dReal dKp = 10, dMax = 100.0;
	dReal dError_Q[MAX_JOINT_NUM];

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	g_tar_q[0] = jointData.Q_tar[0];
	g_tar_q[1] = jointData.Q_tar[1];

	for (int i = 1; i < MAX_JOINT_NUM; i++)
	{
		g_cur_q[i - 1] = dJointGetHingeAngle(g_oJoint[i]);

		if (g_tar_q[i - 1] - g_cur_q[i - 1] > 180.0 * DEG2RAD)
		{
			g_cur_q[i - 1] += 359.9 * DEG2RAD;
		}

		if (g_tar_q[i - 1] - g_cur_q[i - 1] < -180.0 * DEG2RAD)
		{
			g_cur_q[i - 1] -= 359.9 * DEG2RAD;
		}

		dError_Q[i - 1] = g_tar_q[i - 1] - g_cur_q[i - 1];

		dJointSetHingeParam(g_oJoint[i], dParamVel, dKp*dError_Q[i - 1]);
		dJointSetHingeParam(g_oJoint[i], dParamFMax, dMax);
	}

	jointData.Q_cur[0] = g_cur_q[0];
	jointData.Q_cur[1] = g_cur_q[1];

	SET_SYSTEM_MEMORY("JointData", jointData);
	*/
}



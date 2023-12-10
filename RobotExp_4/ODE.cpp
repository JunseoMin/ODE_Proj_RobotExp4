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
	// ��ȣ ��� ���� ��ⷯ ���� ����
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

// �ʱ� ��ġ ���ڷ��� ���� 
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
	// �ؼ��� ���Ѵ�� �� ����
	dsSetColor(0., 0., 0.);
	dGeomCapsuleGetParams(g_oObj[0].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[0].body), dBodyGetRotation(g_oObj[0].body), (float)length, (float)radius);
	// ������ r, ���� L�� Capsule Geometry�� Space �ۼ��ϰ� �� ID��ȣ�� ��ȯ.

	dsSetColor(0.5, 0.5, 0.5);
	dGeomCapsuleGetParams(g_oObj[1].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[1].body), dBodyGetRotation(g_oObj[1].body), (float)length, (float)radius);
	// ������ r, ���� L�� Capsule Geometry�� Space �ۼ��ϰ� �� ID��ȣ�� ��ȯ.

	dsSetColor(1., 1., 1.);
	dGeomCapsuleGetParams(g_oObj[2].geom, &length, &radius);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[2].body), dBodyGetRotation(g_oObj[2].body), (float)length, (float)radius);
	// ������ r, ���� L�� Capsule Geometry�� Space �ۼ��ϰ� �� ID��ȣ�� ��ȯ.

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

	//ĸ���� ������
	dReal r[MAX_JOINT_NUM];

	for (int i = 0; i < MAX_JOINT_NUM; i++)
		r[i] = 0.125;

	//ĸ��(��ũ)�� ����
	// �� ������Ʈ ���ش�� �� ���� ���� 
	dReal length[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

	//ĸ��(��ũ)�� ����
	// �� ������Ʈ ���ش�� ������ ���̿� ���� ���� ���� 
	dReal weight[MAX_JOINT_NUM] = { 0.5, 1.0, 0.5 };

	// Body ����
	//���� �߽���
	// �� ������Ʈ ���ش�� ���� �߽��� ����
	dReal x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal z[MAX_JOINT_NUM] = { 0.25, 1.0, 1.75 };

	//��ũ pose
	// �� ������Ʈ ���ش�� ������ ��ũ �ڼ� ����
	dReal ori_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal ori_z[MAX_JOINT_NUM] = { 1.0, 1.0, 1.0 };
	dReal ori_q[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };

	// Joint ���� 
	//�� ����Ʈ�� ȸ������ ��ġ
	dReal c_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_y[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal c_z[MAX_JOINT_NUM] = { 0.0, 0.5, 1.5 };

	//hindge axis
	// �� ������Ʈ�� ���� �ð������ �ϵ��� ����
	dReal axis_x[MAX_JOINT_NUM] = { 0.0, 0.0, 0.0 };
	dReal axis_y[MAX_JOINT_NUM] = { 0.0, -1.0, -1.0 };
	dReal axis_z[MAX_JOINT_NUM] = { 1.0, 0.0, 0.0 };


	for (int i = 0; i < MAX_JOINT_NUM; i++)
	{
		g_oObj[i].body = dBodyCreate(g_World);								// dBodyCreate �Լ��� �� ��ũ�� �ٵ� ���� -> body�� �����ϰ� id ��ȯ�Ѵ�.
		dBodySetPosition(g_oObj[i].body, x[i], y[i], z[i]);					// �ٵ��� �ʱ� ��ġ ���� -> body�� �ش� ��ġ�� �̵��Ѵ�.
		dMassSetZero(&mass);												// ���� ����ü �ʱ�ȭ
		dMassSetCapsuleTotal(&mass, weight[i], 1, length[i], r[i]);			// dCreateCapsule �Լ��� ������ ������ ����, ������, ���� ������ �����Ѵ�. -> 1�� z��� ��ġ���� ��Ÿ����. 
		dBodySetMass(g_oObj[i].body, &mass);								// �ٵ� ���� ���� ����
		g_oObj[i].geom = dCreateCapsule(g_Space, length[i], r[i]);			// dCreateCapsule �Լ��� �������� r�̰�, ���̰� l�� ĸ�� ����� space�� ��Ÿ���� �̰��� id�� ��ȯ�Ѵ�.
		dGeomSetBody(g_oObj[i].geom, g_oObj[i].body);						// ������Ʈ���� �ٵ� ���� -> body�� ��翡 ���� geometry�� �����Ѵ�.
		dRFromAxisAndAngle(R, ori_x[i], ori_y[i], ori_z[i], ori_q[i]);		// ȸ�� ���� -> rotation matrix�� ȸ�� ��� ȸ�� ������ �Է����� �ִ´�. 
		dBodySetRotation(g_oObj[i].body, R);								// �ٵ� ȸ�� ��Ʈ������ �����Ͽ� ��Ÿ����. 
	}


	// ���� �� ���� 
	g_oJoint[0] = dJointCreateFixed(g_World, 0);						// ���� Joint ����
	dJointAttach(g_oJoint[0], 0, g_oObj[0].body);						// body1�� body2�� ����, Body1 Ȥ�� body2�� ��0���̸� World�� ����
	dJointSetFixed(g_oJoint[0]);

	// Joint ���� 
	for (int i = 1; i < MAX_JOINT_NUM; i++)
	{
		g_oJoint[i] = dJointCreateHinge(g_World, 0);
		dJointAttach(g_oJoint[i], g_oObj[i].body, g_oObj[i - 1].body);
		dJointSetHingeAnchor(g_oJoint[i], c_x[i], c_y[i], c_z[i]);			// Hinge�� �߽���( x, y, z )�� ����
		dJointSetHingeAxis(g_oJoint[i], axis_x[i], axis_y[i], axis_z[i]);	// Hinge�� ȸ���� ����( x, y, z)�� ����
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



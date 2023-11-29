#pragma once

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

typedef struct _dataType {

	double Q_tar[2];
	double Q_cur[2];

}DataType_t;


typedef struct _controlData {

	double position;
	double velocity;
	double current;

	double Kp_cur;
	double Ki_cur;
	double Kp_V;
	double Ki_V;
	double Kp_P;
	double Kd_P;

}ControlData_t;


typedef struct _packet_data {

	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;

	int pos;
	int velo;
	int cur;
	int Kp_cur;
	int Ki_cur;
	int Kp_vel;
	int Ki_vel;
	int Kp_pos;
	int Kd_pos;

}Packet_data_t;


typedef union _packet {

	Packet_data_t data;
	unsigned char buffer[sizeof(Packet_data_t)];

}Packet_t;
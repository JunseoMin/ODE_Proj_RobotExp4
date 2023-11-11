#include "stdafx.h"
#include "DataType.h"
#include "CommWork.h"
#include "SystemMemory.h"

CCommWork::CCommWork(std::string name)
	:CWorkBase(name)
{
	memset(&_target, 0, sizeof(ControlData_t));
	memset(&_current, 0, sizeof(ControlData_t));

	_memname_tar = name + "_Controller_Target";
	_memname_cur = name + "_Controller_Current";
	CREATE_SYSTEM_MEMORY(_memname_tar, ControlData_t);
	CREATE_SYSTEM_MEMORY(_memname_cur, ControlData_t);


}


CCommWork::~CCommWork() {

	DELETE_SYSTEM_MEMORY(_memname_tar);
	DELETE_SYSTEM_MEMORY(_memname_cur);

	ClosePort();
}



bool CCommWork::OpenPort(std::string name, int baudRate) {

	return _comm.Open(name.c_str(), baudRate);
}



void CCommWork::ClosePort() {

	_comm.Close();
}



void CCommWork::_execute() {

	GET_SYSTEM_MEMORY(_memname_tar, _target);

	//TO DO

	SET_SYSTEM_MEMORY(_memname_cur, _current);
}

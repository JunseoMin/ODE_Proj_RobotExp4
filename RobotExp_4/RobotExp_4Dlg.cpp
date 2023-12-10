
// RobotExp_4Dlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"

#include "ThreadWorker.h"
#include "WorkBase.h"
#include "ODEWork.h"

#include "resource.h"

#include "afxdialogex.h"
#include "SystemMemory.h"
#include "DataType.h"
#include <math.h>
#include <chrono>
#include "Comm.h"


#define PI 3.141592

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

double mod(double dividend, double divisor) {
	// 부호 상관 없이 모듈러 연산 수행
	return std::fmod((std::fmod(dividend, divisor) + divisor), divisor);
}


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotExp_4Dlg 대화 상자



CRobotExp_4Dlg::CRobotExp_4Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTEXP_4_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotExp_4Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_PORT, m_ComboPort);
	DDX_Control(pDX, IDC_COMBO_BAUD, m_ComboBaud);
	DDX_Control(pDX, IDC_CHECK_OPEN, m_CheckOpen);
	DDX_Control(pDX, IDC_EDIT_SEND, m_EditSend);
	DDX_Control(pDX, IDC_EDIT2, m_EditRecv);
	DDX_Control(pDX, IDC_EDIT_TAR_POS_1, m_editTarPos1);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_1, m_editCurPos1);
	DDX_Control(pDX, IDC_EDIT_TAR_POS_2, m_editTarPos2);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_2, m_editCurPos2);
	DDX_Control(pDX, IDC_EDIT_TAR_VEL, m_editTarVel);
	DDX_Control(pDX, IDC_EDIT_CUR_VEL, m_editCurVel);
	DDX_Control(pDX, IDC_EDIT_TAR_TOR, m_editTarTorq);
	DDX_Control(pDX, IDC_EDIT_CUR_TOR, m_editCurTorq);
	DDX_Control(pDX, IDC_EDIT_TAR_X, m_editTarX);
	DDX_Control(pDX, IDC_EDIT_CUR_X, m_editCurX);
	DDX_Control(pDX, IDC_EDIT_TAR_Y, m_editTarY);
	DDX_Control(pDX, IDC_EDIT_CUR_Y, m_editCurY);
	DDX_Control(pDX, IDC_EDIT_TAR_Z, m_editTarZ);
	DDX_Control(pDX, IDC_EDIT_CUR_Z, m_editCurZ);
}

BEGIN_MESSAGE_MAP(CRobotExp_4Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_CBN_DROPDOWN(IDC_COMBO_PORT, &CRobotExp_4Dlg::OnCbnDropdownComboPort)
	ON_BN_CLICKED(IDC_CHECK_OPEN, &CRobotExp_4Dlg::OnBnClickedCheckOpen)
	ON_BN_CLICKED(IDC_BTN_SEND, &CRobotExp_4Dlg::OnBnClickedBtnSend)
	ON_BN_CLICKED(IDC_BTN_CLEAR, &CRobotExp_4Dlg::OnBnClickedBtnClear)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_INIT, &CRobotExp_4Dlg::OnBnClickedButtonInit)
	ON_BN_CLICKED(IDC_BUTTON_FORWARD, &CRobotExp_4Dlg::OnBnClickedButtonForward)
	ON_BN_CLICKED(IDC_BUTTON_INVERSE, &CRobotExp_4Dlg::OnBnClickedButtonInverse)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_GRAPH, &CRobotExp_4Dlg::OnBnClickedButtonGraph)
	ON_BN_CLICKED(IDC_BUTTON_SET, &CRobotExp_4Dlg::OnBnClickedButtonSet)
END_MESSAGE_MAP()


// CRobotExp_4Dlg 메시지 처리기

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	SetTimer(1, 100, NULL);

	_commWorker.SetPeriod(0.01);
	_commWorker.SetWork(CreateWork<CCommWork>("Comm1Work"));

	m_editTarPos1.SetWindowText("0");
	m_editTarPos2.SetWindowText("0");

	m_editTarVel.SetWindowText("10");
	m_editTarTorq.SetWindowText("0.1");

	m_editTarX.SetWindowText("0.0");
	m_editTarY.SetWindowText("0.0");
	m_editTarZ.SetWindowText("0.0");

	m_pGraphDlg = new CGraphDlg;
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CRobotExp_4Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CRobotExp_4Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRobotExp_4Dlg::SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos)
{
	const double l1 = 1.;			// Link1의 길이 변수 선언
	const double l2 = 0.5;			// Link2의 길이 변수 선언

	double theta1 = dAngle * PI / 180.0;	// joint의 각도를 theta1라는 변수로 선언
	double theta2 = dAngle2 * PI / 180.0;	// joint의 각도를 theta2라는 변수로 선언 

	// 기하학적인 특성을 이용하여 x, y, z에 대한 값을 나타낸다.
	pdPos[0] = l1 * cos(theta1) + l2 * cos(theta1 + theta2);	// x축 위치
	pdPos[1] = l1 * sin(theta1) + l2 * sin(theta1 + theta2);	// y축 위치
	pdPos[2] = 0;												// z축 위치
}


void CRobotExp_4Dlg::SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle)
{
	const double l1 = 1.;
	const double l2 = 0.5;

	double costheta2 = (dX * dX + dY * dY - l1 * l1 - l2 * l2) / (2 * l1 * l2);
	double sintheta2 = sqrt(1 - costheta2 * costheta2);

	pdAngle[1] = atan2(sintheta2, costheta2);

	double theta2 = atan2(sintheta2, costheta2);


	double k1 = l1 + l2 * cos(theta2);
	double k2 = l2 * sin(theta2);

	double costheta1 = (k1 * dX + k2 * dY) / (k1 * k1 + k2 * k2);
	double sintheta1 = (k1 * dY - k2 * dX) / (k1 * k1 + k2 * k2);

	pdAngle[0] = atan2(sintheta1, costheta1);	// 구한 값을 이용하여 동일하게 theta1을 구한다. 
}

void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDeviceListReader reader;
	std::vector<std::string> list;

	//Combo Box 초기화
	m_ComboPort.ResetContent();

	//컴퓨터에 연결된 시리얼 장비 리스트를 얻어옴
	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);

	//Combo Box에 list 추가
	for (int i = 0; i < list.size(); i++)
	{
		m_ComboPort.AddString(list[i].c_str());
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckOpen()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//버튼의 체크 여부 확인 
	if (m_CheckOpen.GetCheck())
	{
		//체크 된 경우
		//선택된 포트 이름과 통신속도를 가져옴
		CString port, baud;
		m_ComboPort.GetLBText(m_ComboPort.GetCurSel(), port);
		m_ComboBaud.GetLBText(m_ComboBaud.GetCurSel(), baud);

		int nTmp = atoi(baud.GetBuffer());
		ControlData_t motor_data;
		
		// 포트 열기 시도
		if (((CCommWork*)_commWorker.GetWork())->OpenPort(port.GetBuffer(), nTmp))
		{
			// 성공하면 Thread를 만들고 버튼의 텍스트를 Close로 변경
			_commWorker.StartWork();
			m_CheckOpen.SetWindowText("Close");

			motor_data.position = 0.;
			motor_data.velocity = 10 * DEG2RAD;
			motor_data.current = 0.1 / 0.0683;

			SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);
		}
		else
		{
			// 실패 시 에러 메세지를 띄우고 초기화
			AfxMessageBox("Can`t open port");
			m_CheckOpen.SetCheck(false);
		}
	}
	else
	{
		// 체크 해제된 경우
		// 포트를 닫고 버튼 텍스트를 Open으로 변경
		_commWorker.StopWork();
		((CCommWork*)_commWorker.GetWork())->ClosePort();
		m_CheckOpen.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnBnClickedBtnSend()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.가

}


void CRobotExp_4Dlg::OnBnClickedBtnClear()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_EditRecv.SetWindowText("");
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	ControlData_t motor_data;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data);

	CString str;

	//ODE
	str.Format("%.4f", ode_data.Q_cur[1]);
	m_editCurPos2.SetWindowText(str);
	
	//Motor
	str.Format("%.4f", fmod(motor_data.position * RAD2DEG,360.));		
	// ode 실행시 current 가 안나오는게 맞음, 원하면 ode_data.Qcur[1]로 설정
	
	m_editCurPos1.SetWindowText(str);

	str.Format("%.4f", motor_data.velocity * RAD2DEG);
	m_editCurVel.SetWindowText(str);

	str.Format("%.4f", motor_data.current * 0.0683);
	m_editCurTorq.SetWindowText(str);

	//Forward Kinematics
	double Pcur[3] = { 0, };
	SolveForwardKinematics(ode_data.Q_cur[0], ode_data.Q_cur[1], Pcur);

	str.Format("%.4f", Pcur[0]);
	m_editCurX.SetWindowText(str);

	str.Format("%.4f", Pcur[1]);
	m_editCurY.SetWindowText(str);

	str.Format("%.4f", Pcur[2]);
	m_editCurZ.SetWindowText(str);

	/*
	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", jointData.Q_cur[0] * RAD2DEG);
	m_editCurPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", jointData.Q_cur[1] * RAD2DEG);
	m_editCurPos2.SetWindowTextA(pszTmp);

	double dTmp[3] = { 0,0,0 };
	SolveForwardKinematics(jointData.Q_cur[0] * RAD2DEG, jointData.Q_cur[1] * RAD2DEG, dTmp);
	sprintf_s(pszTmp, "%.2lf", dTmp[0]);
	m_editCurX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dTmp[1]);
	m_editCurY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dTmp[2]);
	m_editCurZ.SetWindowTextA(pszTmp);
	*/

	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnBnClickedButtonInit()	// ODE 및 실제 Motor의 Setting을 초기화
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	m_editTarX.SetWindowText("0");
	m_editTarY.SetWindowText("0");
	m_editTarZ.SetWindowText("0");

	m_editTarPos1.SetWindowText("0");
	m_editTarPos2.SetWindowText("0");

	m_editTarVel.SetWindowText("10");
	m_editTarTorq.SetWindowText("0.1");

	ControlData_t motor_data;
	DataType_t ode_data;
	GET_SYSTEM_MEMORY("JointData", ode_data);

	ode_data.Q_tar[0] = ode_data.Q_tar[1] = 0.0;

	SET_SYSTEM_MEMORY("JointData", ode_data);

	motor_data.position = 0.;
	motor_data.velocity = 10 * DEG2RAD;
	motor_data.current = 0.1 / 0.0683;

	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);

	/*
	m_editTarPos1.SetWindowTextA("0");
	m_editTarPos2.SetWindowTextA("0");
	m_editTarVel.SetWindowTextA("0");
	m_editTarTorq.SetWindowTextA("0");
	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0");
	m_editTarZ.SetWindowTextA("0.0");
	*/
}


void CRobotExp_4Dlg::OnBnClickedButtonForward(){
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	char _tempChar[10];
	double d0, d1 = 0.;

	m_editTarPos1.GetWindowTextA(_tempChar, 10);
	d0 = atof(_tempChar);
	m_editTarPos2.GetWindowTextA(_tempChar, 10);
	d1 = atof(_tempChar);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	d1 = mod(d1, 360.);

	// exception
	if (d0 >= 360) { 
		d0 -= 360; 
		d0 = mod(d0, 360.);
	}
	else if (d0 == 180) { 
		d0 = d0 - 0.01; 
		d0 = mod(d0, 360.);
	}
	else if (d0 > 180) { 
		d0 -= 360; 
		d0 = mod(d0, 360.);
	}

	if (d0 <= -360) { 
		d0 += 360;
		d0 = mod(d0, 360.);
	}
	else if (d0 == -180) {
		d0 = d0 + 0.01;
		d0 = mod(d0, 360.);
	}
	else if (d0 < -180) {
		d0 += 360;
		d0 = mod(d0, 360.);
	}

	jointData.Q_tar[0] = d0;

	if (d1 >= 360) {
		d1 -= 360;
		d1 = mod(d1, 360.);
	}
	else if (d1 == 180) {
		d1 = d1 - 0.01;
		d1 = mod(d1, 360.);
	}
	else if (d1 > 180) {
		d1 -= 360;
		d1 = mod(d1, 360.);
	}

	if (d1 <= -360) {
		d1 += 360;
		d1 = mod(d1, 360.);
	}
	else if (d1 == -180) {
		d1 = d1 + 0.01;
		d1 = mod(d1, 360.);
	}
	else if (d1 < -180) {
		d1 += 360;
		d1 = mod(d1, 360.);
	}
	jointData.Q_tar[1] = d1;


	SET_SYSTEM_MEMORY("JointData", jointData);	// set AVR

	double dPos[3] = { 0., };

	SolveForwardKinematics(d0, d1, dPos);

	char pszTmp[512];

	sprintf_s(pszTmp, "%.4f", dPos[0]);
	m_editTarX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.4f", dPos[1]);
	m_editTarY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.4f", dPos[2]);
	m_editTarZ.SetWindowTextA(pszTmp);
	/*
	char cTmp[10];
	double dTmp[2];
	m_editTarPos1.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarPos2.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	jointData.Q_tar[0] = dTmp[0] * DEG2RAD;
	jointData.Q_tar[1] = dTmp[1] * DEG2RAD;

	SET_SYSTEM_MEMORY("JointData", jointData);

	double dPos[3] = { 0, 0, 0 };

	SolveForwardKinematics(dTmp[0], dTmp[1], dPos);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dPos[0]);
	m_editTarX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[1]);
	m_editTarY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[2]);
	m_editTarZ.SetWindowTextA(pszTmp);
	*/
}


void CRobotExp_4Dlg::OnBnClickedButtonInverse()	
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	char _tempChar[10];
	double _tempDouble[3];
	m_editTarX.GetWindowTextA(_tempChar, 10);
	_tempDouble[0] = atof(_tempChar);
	m_editTarY.GetWindowTextA(_tempChar, 10);
	_tempDouble[1] = atof(_tempChar);
	m_editTarZ.GetWindowTextA(_tempChar, 10);
	_tempDouble[2] = atof(_tempChar);

	double dAngle[2] = { 0, 0 };
	SolveInverseKinematics(_tempDouble[0], _tempDouble[1], _tempDouble[2], dAngle);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2f", dAngle[0] * RAD2DEG);
	m_editTarPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2f", dAngle[1] * RAD2DEG);
	m_editTarPos2.SetWindowTextA(pszTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	jointData.Q_tar[0] = dAngle[0] * RAD2DEG;
	jointData.Q_tar[1] = dAngle[1] * RAD2DEG;

	SET_SYSTEM_MEMORY("JointData", jointData);
	/*
	char cTmp[10];
	double dTmp[3];
	m_editTarX.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarY.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);
	m_editTarZ.GetWindowTextA(cTmp, 10);
	dTmp[2] = atof(cTmp);

	double dAngle[2] = { 0, 0 };
	SolveInverseKinematics(dTmp[0], dTmp[1], dTmp[2], dAngle);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dAngle[0] * RAD2DEG);
	m_editTarPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dAngle[1] * RAD2DEG);
	m_editTarPos2.SetWindowTextA(pszTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	jointData.Q_tar[0] = dAngle[0];
	jointData.Q_tar[1] = dAngle[1];

	SET_SYSTEM_MEMORY("JointData", jointData);
	*/
}


void CRobotExp_4Dlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	delete m_pGraphDlg;
}


void CRobotExp_4Dlg::OnBnClickedButtonGraph()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	BOOL bCheck = m_pGraphDlg->IsWindowVisible();

	if (bCheck) m_pGraphDlg->ShowWindow(SW_HIDE);

	else m_pGraphDlg->ShowWindow(SW_SHOW);
}


void CRobotExp_4Dlg::OnBnClickedButtonSet()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	ControlData_t motor_data;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);

	CString str;

	char _tempChar[10];
	double d0, d1 = 0.;

	m_editTarPos1.GetWindowTextA(_tempChar, 10);
	d0 = atof(_tempChar);

	// exception
	if (d0 >= 360) {
		d0 -= 360;
		d0 = mod(d0, 360.);
	}
	else if (d0 == 180) {
		d0 = d0 - 0.01;
		d0 = mod(d0, 360.);
	}
	else if (d0 > 180) {
		d0 -= 360;
		d0 = mod(d0, 360.);
	}

	if (d0 <= -360) {
		d0 += 360;
		d0 = mod(d0, 360.);
	}
	else if (d0 == -180) {
		d0 = d0 + 0.01;
		d0 = mod(d0, 360.);
	}
	else if (d0 < -180) {
		d0 += 360;
		d0 = mod(d0, 360.);
	}
	ode_data.Q_tar[0] = d0;

	m_editTarPos2.GetWindowText(str);
	ode_data.Q_tar[1] = atof(str.GetBuffer());

	//ODE
	str.Format("%.4f", ode_data.Q_cur[1]);
	m_editCurPos2.SetWindowText(str);

	m_editTarPos1.GetWindowText(str);
	motor_data.position = d0*DEG2RAD;

	m_editTarVel.GetWindowText(str);
	motor_data.velocity = atof(str.GetBuffer()) * DEG2RAD;

	m_editTarTorq.GetWindowText(str);
	motor_data.current = atof(str.GetBuffer()) / 0.0683;

	SET_SYSTEM_MEMORY("JointData", ode_data);
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);

}

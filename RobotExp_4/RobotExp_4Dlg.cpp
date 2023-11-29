
// RobotExp_4Dlg.cpp : ���� ����
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"
#include "afxdialogex.h"
#include "DataType.h"
#include <math.h>
#include "SystemMemory.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
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


// CRobotExp_4Dlg ��ȭ ����



CRobotExp_4Dlg::CRobotExp_4Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTEXP_4_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

BEGIN_MESSAGE_MAP(CRobotExp_4Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
//	ON_BN_CLICKED(IDC_CHECK1, &CRobotExp_4Dlg::OnBnClickedCheck1)
	ON_CBN_DROPDOWN(IDC_COMBO_PORT, &CRobotExp_4Dlg::OnCbnDropdownComboPort)
	ON_BN_CLICKED(IDC_CHECK1_OPEN, &CRobotExp_4Dlg::OnBnClickedCheck1Open)
	ON_EN_CHANGE(IDC_EDIT_RECV, &CRobotExp_4Dlg::OnEnChangeEditRecv)
	ON_BN_CLICKED(IDC_BTN_SEND, &CRobotExp_4Dlg::OnBnClickedBtnSend)
	ON_BN_CLICKED(IDC_BTN_CLEAR, &CRobotExp_4Dlg::OnBnClickedBtnClear)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON1, &CRobotExp_4Dlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CRobotExp_4Dlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CRobotExp_4Dlg::OnBnClickedButton3)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_GRAPH, &CRobotExp_4Dlg::OnBnClickedButtonGraph)
	ON_BN_CLICKED(IDC_BUTTON_Set, &CRobotExp_4Dlg::OnBnClickedButtonSet)
END_MESSAGE_MAP()


// CRobotExp_4Dlg �޽��� ó����

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
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

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	_commWorker.SetPeriod(0.01);
	_commWorker.SetWork(CreateWork<CCommWork>("CommWork"));
		
	m_editTarPos1.SetWindowTextA("0");
	m_editTarPos1.SetWindowTextA("0");

	m_editTarVel.SetWindowTextA("10");
	m_editTarTorq.SetWindowTextA("0.1");

	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0");
	m_editTarZ.SetWindowTextA("2.0");


	// set graph dialog button
	m_pGraphDlg = new CGraphDlg();
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);
	
	SetTimer(1001, 33, NULL);

	// set data


	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
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

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CRobotExp_4Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRobotExp_4Dlg::SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos)
{

	double L1 = 1;
	double L2 = 0.5;

	pdPos[0] = L1 * sin(dAngle * DEG2RAD) + L2 * sin(dAngle * DEG2RAD + dAngle2 * DEG2RAD);
	pdPos[1] = 0;
	pdPos[2] = 0.5 + L1 * cos(dAngle * DEG2RAD) + L2 * cos(dAngle * DEG2RAD + dAngle2 * DEG2RAD);

}


void CRobotExp_4Dlg::SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle)
{
	double L1 = 1;
	double L2 = 0.5;

	//base offset
	dZ = dZ - 0.5;

	double c2 = (pow(dZ, 2) + pow(dX, 2) - (pow(L1, 2) + pow(L2, 2)))
		/ (2 * L1 * L2);
	double s2 = abs(sqrt(1 - pow(c2, 2)));				

	double q2 = atan2(s2, c2);

	double c1 = ((L1 + L2 * c2) * dZ + L2 * s2 * dX)
		/ (pow(L1 + L2 * c2, 2) + pow(L2 * s2, 2));
	double s1 = (-L2 * s2 * dZ + (L1 + L2 * c2) * dX)
		/ (pow(L1 + L2 * c2, 2) + pow(L2 * s2, 2));

	double q1 = atan2(s1, c1);

	// theta 1
	pdAngle[0] = q1 * RAD2DEG;
	// theta 2
	pdAngle[1] = q2 * RAD2DEG;
}

void CRobotExp_4Dlg::OnBnClickedCheck1()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
}


void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CDeviceListReader reader;
	std::vector<std::string> list;

	//Combo box initialization
	m_ComboPort.ResetContent();

	// get serial tool list
	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);

	// add to combobox
	for (int i = 0; i < list.size(); i++)
	{
		m_ComboPort.AddString(list[i].c_str());
	}

}


void CRobotExp_4Dlg::OnBnClickedCheck1Open()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	//check button
	if (m_CheckOpen.GetCheck())
	{
		CString port, baud;

		m_ComboPort.GetLBText(m_ComboPort.GetCurSel(), port);
		m_ComboBaud.GetLBText(m_ComboBaud.GetCurSel(), baud);
		int nTmp = atoi(baud.GetBuffer());

		if (((CCommWork*)_commWorker.GetWork())->OpenPort(port.GetBuffer(), nTmp))
		{
			_commWorker.StartWork();
			m_CheckOpen.SetWindowText("Close");
		}
		else
		{
			AfxMessageBox("Can't open port");
			m_CheckOpen.SetCheck(false);
		}
	}
	else
	{
		_commWorker.StopWork();
		((CCommWork*)_commWorker.GetWork())->ClosePort();
		m_CheckOpen.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnEnChangeEditRecv()
{
	// TODO:  RICHEDIT ��Ʈ���� ���, �� ��Ʈ����
	// CDialogEx::OnInitDialog() �Լ��� ������ 
	//�ϰ� ����ũ�� OR �����Ͽ� ������ ENM_CHANGE �÷��׸� �����Ͽ� CRichEditCtrl().SetEventMask()�� ȣ������ ������
	// �� �˸� �޽����� ������ �ʽ��ϴ�.

	// TODO:  ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
}


void CRobotExp_4Dlg::OnBnClickedBtnSend()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	/*
	if (m_comm.isOpen())
	{
		CString str;
		m_EditSend.GetWindowText(str);

		int size = m_comm.Write(str.GetBuffer(), str.GetLength());

		m_EditSend.SetWindowText("");
	}
	*/


}


void CRobotExp_4Dlg::OnBnClickedBtnClear()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_EditRecv.SetWindowText("");
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	ControlData_t motor_data;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);
	GET_SYSTEM_MEMORY("CommWork_Controller_Current",motor_data);

	CString str;
	
	// ODE settings
	str.Format("%.4f", ode_data.Q_cur[0]);
	m_editCurPos1.SetWindowText(str);

	str.Format("%.4f", ode_data.Q_cur[1]);
	m_editCurPos2.SetWindowText(str);

	// 
	//str.Format("%.4f", motor_data.position * RAD2DEG);
	//m_editCurPos1.SetWindowText(str);

	//velocity
	str.Format("%.4f", motor_data.velocity* RAD2DEG);
	m_editCurVel.SetWindowText(str);

	//Motor Torque
	str.Format("%.4f", motor_data.current * 0.0683);
	m_editCurTorq.SetWindowText(str);

	//Foward Kinematics
	double Pcur[3] = { 0, };
	SolveForwardKinematics(ode_data.Q_cur[0], ode_data.Q_cur[1], Pcur);

	str.Format("%.4f", Pcur[0]);
	m_editCurX.SetWindowText(str);

	str.Format("%.4f", Pcur[1]);
	m_editCurY.SetWindowText(str);

	str.Format("%.4f", Pcur[2]);
	m_editCurZ.SetWindowText(str);

	// set joint data
	SET_SYSTEM_MEMORY("JointData", ode_data);

	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnBnClickedButton1()
{
	// init button
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_editTarPos1.SetWindowTextA("0.0");	//set target
	m_editTarPos2.SetWindowTextA("0.0");
	m_editTarVel.SetWindowTextA("10.0");
	m_editTarTorq.SetWindowTextA("0.1");

	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0");
	m_editTarZ.SetWindowTextA("2.0");

	// gain settings
	m_edit_Kp_cur.SetWindowTextA("1.3");		
	m_edit_Ki_cur.SetWindowTextA("40");
	m_edit_kp_V.SetWindowTextA("3");
	m_edit_Ki_V.SetWindowTextA("3.5");
	m_edit_Kp_P.SetWindowTextA("1.5");
	m_edit_Kd_P.SetWindowTextA("0.01");

	CString str;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);

	ode_data.Q_tar[0] = ode_data.Q_tar[1] = 0.;

	SET_SYSTEM_MEMORY("JointData", ode_data);

	ControlData_t motor_data;

	m_editTarPos1.GetWindowText(str);
	motor_data.position = atof(str.GetBuffer())*DEG2RAD;

	m_editTarVel.GetWindowText(str);
	motor_data.velocity = atof(str.GetBuffer())*DEG2RAD;

	m_editTarTorq.GetWindowText(str);
	motor_data.current = atof(str.GetBuffer())/0.0683;

	// set gain by ctr data
	m_edit_Kp_cur.GetWindowText(str);								
	motor_data.Kp_cur = atof(str.GetBuffer());

	m_edit_Ki_cur.GetWindowText(str);
	motor_data.Ki_cur = atof(str.GetBuffer());

	m_edit_kp_V.GetWindowText(str);
	motor_data.Kp_V = atof(str.GetBuffer());

	m_edit_Ki_V.GetWindowText(str);
	motor_data.Ki_V = atof(str.GetBuffer());

	m_edit_Kp_P.GetWindowText(str);
	motor_data.Kp_P = atof(str.GetBuffer());

	m_edit_Kd_P.GetWindowText(str);
	motor_data.Kd_P = atof(str.GetBuffer());
	
	// set to memory
	SET_SYSTEM_MEMORY("CommWork_Controller_Current", motor_data);
}


void CRobotExp_4Dlg::OnBnClickedButton2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	// foward button

	char cTmp[10];
	double dTmp[2];
	m_editTarPos1.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarPos2.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	// Gimbol lock exception
	if (dTmp[0] >= 360)	dTmp[0] -= 360;
	else if (dTmp[0] == 180) dTmp[0] = dTmp[0] - 0.01;
	else if (dTmp[0] > 180) dTmp[0] -= 360;

	if (dTmp[0] <= -360) dTmp[0] += 360;
	else if (dTmp[0] == -180) dTmp[0] = dTmp[0] + 0.01;
	else if (dTmp[0] < -180) dTmp[0] += 360;

	jointData.Q_tar[0] = dTmp[0];

	if (dTmp[1] >= 360)	dTmp[1] -= 360;
	else if (dTmp[1] == 180) dTmp[1] = dTmp[1] - 0.01;
	else if (dTmp[1] > 180) dTmp[1] -= 360;

	if (dTmp[1] <= -360) dTmp[1] += 360;
	else if (dTmp[1] == -180) dTmp[1] = dTmp[1] + 0.01;
	else if (dTmp[1] < -180) dTmp[1] += 360;
	jointData.Q_tar[1] = dTmp[1];

	SET_SYSTEM_MEMORY("JointData", jointData);

	double dPos[3] = { 0, 0, 0 };

	SolveForwardKinematics(dTmp[0], dTmp[1], dPos);

	char pszTmp[512];

	sprintf_s(pszTmp, "%.2f", dPos[0]);
	m_editTarX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2f", dPos[1]);
	m_editTarY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2f", dPos[2]);
	m_editTarZ.SetWindowTextA(pszTmp);
}


void CRobotExp_4Dlg::OnBnClickedButton3()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	// backward button
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
	sprintf_s(pszTmp, "%.2f", dAngle[0]);
	m_editTarPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2f", dAngle[1]);
	m_editTarPos2.SetWindowTextA(pszTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	jointData.Q_tar[0] = dAngle[0];
	jointData.Q_tar[1] = dAngle[1];

	SET_SYSTEM_MEMORY("JointData", jointData);

}


void CRobotExp_4Dlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	delete m_pGraphDlg;
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
}


void CRobotExp_4Dlg::OnBnClickedButtonGraph()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	BOOL bCheck = m_pGraphDlg->IsWindowVisible();
	if (bCheck) m_pGraphDlg->ShowWindow(SW_HIDE);
	else m_pGraphDlg->ShowWindow(SW_SHOW);
	
}


void CRobotExp_4Dlg::OnBnClickedButtonSet()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	ControlData_t motor_data;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);

	CString str;
	m_editCurPos1.GetWindowText(str);
	ode_data.Q_tar[0] = atof(str.GetBuffer())*DEG2RAD;

	m_editCurPos2.GetWindowText(str);
	ode_data.Q_tar[1] = atof(str.GetBuffer())*DEG2RAD;


	m_editTarPos1.GetWindowText(str);
	motor_data.position = atof(str.GetBuffer())*DEG2RAD;

	m_editTarVel.GetWindowText(str);
	motor_data.velocity = atof(str.GetBuffer())*DEG2RAD;

	m_editTarTorq.GetWindowText(str);
	motor_data.current = atof(str.GetBuffer()) / 0.0683;

	m_edit_Kp_cur.GetWindowText(str);
	motor_data.Kp_cur = atof(str.GetBuffer());

	m_edit_Ki_cur.GetWindowText(str);
	motor_data.Ki_cur = atof(str.GetBuffer());

	m_edit_kp_V.GetWindowText(str);
	motor_data.Kp_V = atof(str.GetBuffer());

	m_edit_Ki_V.GetWindowText(str);
	motor_data.Ki_V = atof(str.GetBuffer());

	m_edit_Kp_P.GetWindowText(str);
	motor_data.Kp_P = atof(str.GetBuffer());

	m_edit_Kd_P.GetWindowText(str);
	motor_data.Kd_P = atof(str.GetBuffer());

	SET_SYSTEM_MEMORY("JointData", ode_data);
	SET_SYSTEM_MEMORY("CommWork_Controller_Current", motor_data);

	/////////////////////////////////////////////
}

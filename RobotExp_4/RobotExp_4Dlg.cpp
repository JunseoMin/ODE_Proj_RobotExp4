
// RobotExp_4Dlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"
#include "afxdialogex.h"
#include "DataType.h"
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


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
	DDX_Control(pDX, IDC_CHECK1_OPEN, m_CheckOpen);
	DDX_Control(pDX, IDC_EDIT_SEND, m_EditSend);
	DDX_Control(pDX, IDC_EDIT_RECV, m_EditRecv);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_1, m_editCurPos1);
	DDX_Control(pDX, IDC_EDIT_TAR_POS_1, m_editTarPos1);
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
	SetTimer(1001, 33, NULL);

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
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
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
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//check button
	if (m_CheckOpen.GetCheck())
	{
		CString port, baud;
		m_ComboPort.GetLBText(m_ComboPort.GetCurSel(), port);
		m_ComboBaud.GetLBText(m_ComboBaud.GetCurSel(), baud);
		if (m_comm.Open(port.GetBuffer(),atoi(baud.GetBuffer())))
		{
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
		m_comm.Close();
		m_CheckOpen.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnEnChangeEditRecv()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CRobotExp_4Dlg::OnBnClickedBtnSend()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if (m_comm.isOpen())
	{
		CString str;
		m_EditSend.GetWindowText(str);

		int size = m_comm.Write(str.GetBuffer(), str.GetLength());

		m_EditSend.SetWindowText("");
	}
}


void CRobotExp_4Dlg::OnBnClickedBtnClear()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_EditRecv.SetWindowText("");
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if (m_comm.isOpen())
	{
		CString str;
		char buf[4096] = { 0, };

		m_EditRecv.GetWindowText(str);
		str == buf;

		m_EditRecv.SetWindowText(str);
	}

	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnBnClickedButton1()
{
	// init button
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CRobotExp_4Dlg::OnBnClickedButton2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	//foward button
}


void CRobotExp_4Dlg::OnBnClickedButton3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	// backward button
}


void CRobotExp_4Dlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
}

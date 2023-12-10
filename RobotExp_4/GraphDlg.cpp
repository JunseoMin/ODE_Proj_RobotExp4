// GraphDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "GraphDlg.h"
#include "afxdialogex.h"
#include "SystemMemory.h"
#include "DataType.h"
#include "resource.h"


// CGraphDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CGraphDlg, CDialogEx)

CGraphDlg::CGraphDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GRAPH_DIALOG, pParent)
{

}

CGraphDlg::~CGraphDlg()
{
}

void CGraphDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_NTGRAPH_POS, m_ntgPos);
	DDX_Control(pDX, IDC_NTGRAPH_VEL, m_nygVel);
	DDX_Control(pDX, IDC_NTGRAPH_TORQ, m_ntgTor);
}

BOOL CGraphDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	InitNTGraph();

	return TRUE;
	// return TRUE unless you set the force to a control
	// 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}

void CGraphDlg::InitNTGraph()
{
	m_ntgPos.ClearGraph();
	m_nygVel.ClearGraph();
	m_ntgTor.ClearGraph();

	m_ntgPos.SetFrameStyle(0);
	m_nygVel.SetFrameStyle(0);
	m_ntgTor.SetFrameStyle(0);

	m_ntgPos.SetPlotAreaColor(WHITE);
	m_nygVel.SetPlotAreaColor(WHITE);
	m_ntgTor.SetPlotAreaColor(WHITE);

	m_ntgPos.SetShowGrid(TRUE);
	m_nygVel.SetShowGrid(TRUE);
	m_ntgTor.SetShowGrid(TRUE);

	m_ntgPos.SetFormatAxisBottom(_T("%.2f"));
	m_nygVel.SetFormatAxisBottom(_T("%.2f"));
	m_ntgTor.SetFormatAxisBottom(_T("%.2f"));

	m_ntgPos.SetCaption(_T("위치"));
	m_nygVel.SetCaption(_T("속도"));
	m_ntgTor.SetCaption(_T("토크"));

	m_ntgPos.SetXLabel(_T("Time[s]"));
	m_nygVel.SetXLabel(_T("Time[s]"));
	m_ntgTor.SetXLabel(_T("Time[s]"));

	m_ntgPos.SetYLabel(_T("Degree[deg]"));
	m_nygVel.SetYLabel(_T("Velocity[deg/s]"));
	m_ntgTor.SetYLabel(_T("Torque[Nm]"));

	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(RED);// Target

	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(BLUE); // Current

	m_ntgPos.SetRange(0.0, 10.0, 0.0, 360.0);
	m_ntgPos.SetYGridNumber(4);

	m_nygVel.AddElement();
	m_nygVel.SetElementWidth(4);
	m_nygVel.SetElementLineColor(RED);// Target

	m_nygVel.AddElement();
	m_nygVel.SetElementWidth(3);
	m_nygVel.SetElementLineColor(BLUE); // Current

	m_ntgTor.AddElement();
	m_ntgTor.SetElementWidth(4);
	m_ntgTor.SetElementLineColor(RED);// Target

	m_ntgTor.AddElement();
	m_ntgTor.SetElementWidth(3);
	m_ntgTor.SetElementLineColor(BLUE); // Current

	SetTimer(1, 100, NULL);
}



BEGIN_MESSAGE_MAP(CGraphDlg, CDialogEx)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// CGraphDlg 메시지 처리기입니다.


void CGraphDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	m_dCnt += 0.1;
	DataType_t jointData;
	ControlData_t motor_data_tar;
	ControlData_t motor_data_cur;

	GET_SYSTEM_MEMORY("JointData", jointData);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data_cur);

	if (m_dCnt >= 10.0)
	{
		m_ntgPos.SetRange(m_dCnt - 10.0, m_dCnt, 0.0, 360.0);
		m_nygVel.SetRange(m_dCnt - 10.0, m_dCnt, -50.0, 50.0);
		m_ntgTor.SetRange(m_dCnt - 10.0, m_dCnt, -0.2, 0.2);
	}

	m_ntgPos.PlotXY(m_dCnt, motor_data_tar.position * RAD2DEG, 1);
	m_ntgPos.PlotXY(m_dCnt, motor_data_cur.position * RAD2DEG, 2);

	m_nygVel.PlotXY(m_dCnt, motor_data_tar.velocity * RAD2DEG, 1);
	m_nygVel.PlotXY(m_dCnt, motor_data_cur.velocity * RAD2DEG, 2);

	m_ntgTor.PlotXY(m_dCnt, motor_data_tar.current * 0.0683, 1);
	m_ntgTor.PlotXY(m_dCnt, motor_data_cur.current * 0.0683, 2);

	CDialogEx::OnTimer(nIDEvent);
}

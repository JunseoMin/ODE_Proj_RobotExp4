
// RobotExp_4Dlg.h : 헤더 파일
//

#pragma once
#include "Comm.h"
#include "CommWork.h"
#include "DeviceListReader.h"

#include "GraphDlg.h"

#include "ThreadWorker.h"
#include "SharedMemory.h"
#include "SystemMemory.h"
#include "DataType.h"

// CRobotExp_4Dlg 대화 상자
class CRobotExp_4Dlg : public CDialogEx
{
// 생성입니다.
public:
	CRobotExp_4Dlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ROBOTEXP_4_DIALOG };
#endif

	protected:
		virtual void DoDataExchange(CDataExchange* pDX)
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
			DDX_Control(pDX, IDC_EDIT_Kp_cur, m_edit_Kp_cur);
			DDX_Control(pDX, IDC_EDIT_Ki_cur, m_edit_Ki_cur);
			DDX_Control(pDX, IDC_EDIT_Kp_V, m_edit_kp_V);
			DDX_Control(pDX, IDC_EDIT_Ki_V, m_edit_Ki_V);
			DDX_Control(pDX, IDC_EDIT_Kp_P, m_edit_Kp_P);
			DDX_Control(pDX, IDC_EDIT_Kd_P, m_edit_Kd_P);
		}	// DDX/DDV 지원입니다.


public:
	void SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos);
	void SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle);

// 구현입니다.
protected:
	HICON m_hIcon;
	CComm m_comm;
	CThreadedWorker _commWorker;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCheck1();
	CComboBox m_ComboPort;
	CComboBox m_ComboBaud;
	CButton m_CheckOpen;
	afx_msg void OnCbnDropdownComboPort();
	afx_msg void OnBnClickedCheck1Open();
	CEdit m_EditSend;
	afx_msg void OnEnChangeEditRecv();
	CEdit m_EditRecv;
	afx_msg void OnBnClickedBtnSend();
	afx_msg void OnBnClickedBtnClear();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CEdit m_editCurPos1;
	CEdit m_editTarPos1;
	CEdit m_editTarPos2;
	CEdit m_editCurPos2;
	CEdit m_editTarVel;
	CEdit m_editCurVel;
	CEdit m_editTarTorq;
	CEdit m_editCurTorq;
	CEdit m_editTarX;
	CEdit m_editCurX;
	CEdit m_editTarY;
	CEdit m_editCurY;
	CEdit m_editTarZ;
	CEdit m_editCurZ;
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnDestroy();

private:
	CGraphDlg* m_pGraphDlg;

public:
	afx_msg void OnBnClickedButtonGraph();
	afx_msg void OnBnClickedButtonSet();
	CEdit m_edit_Kp_cur;
	CEdit m_edit_Ki_cur;
	CEdit m_edit_kp_V;
	CEdit m_edit_Ki_V;
	CEdit m_edit_Kp_P;
	CEdit m_edit_Kd_P;
};

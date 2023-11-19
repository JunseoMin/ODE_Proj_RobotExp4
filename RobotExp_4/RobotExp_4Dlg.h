
// RobotExp_4Dlg.h : ��� ����
//

#pragma once
#include "Comm.h"
#include "CommWork.h"
#include "DeviceListReader.h"
#include "GraphDlg.h"

// CRobotExp_4Dlg ��ȭ ����
class CRobotExp_4Dlg : public CDialogEx
{
// �����Դϴ�.
public:
	CRobotExp_4Dlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ROBOTEXP_4_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


public:
	void SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos);
	void SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle);

// �����Դϴ�.
protected:
	HICON m_hIcon;
	CComm m_comm;

	// ������ �޽��� �� �Լ�
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
};

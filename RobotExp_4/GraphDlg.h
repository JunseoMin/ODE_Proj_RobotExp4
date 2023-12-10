#pragma once
#include "NTGraph.h"
//#include "COCX.h"

#define RED      RGB(127, 0, 0)
#define GREEN   RGB(0, 127, 0)
#define BLUE   RGB(0, 0, 127)
#define BLACK   RGB(0, 0, 0)
#define WHITE   RGB(255, 255, 255)
#define GRAY   RGB(192, 192, 192)

#define DEG2RAD 0.0174533   // ������ ��������
#define RAD2DEG 57.2958      // ������ ������

// CGraphDlg ��ȭ �����Դϴ�.

class CGraphDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CGraphDlg)

public:
	CGraphDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CGraphDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GRAPH_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.
	virtual void InitNTGraph();
	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	double m_dCnt = 0;

	CNTGraph m_ntgPos;
	CNTGraph m_nygVel;
	CNTGraph m_ntgTor;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

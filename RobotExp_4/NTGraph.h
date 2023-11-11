#pragma once
#include "_Picture.h"
#include "_Font.h"

// 컴퓨터에서 Microsoft Visual C++를 사용하여 생성한 IDispatch 래퍼 클래스입니다.

// 참고: 이 파일의 내용을 수정하지 마십시오.  Microsoft Visual C++에서
//  이 클래스를 다시 생성할 때 수정한 내용을 덮어씁니다.

/////////////////////////////////////////////////////////////////////////////
// CNTGraph 래퍼 클래스입니다.

class CNTGraph : public CWnd
{
protected:
	DECLARE_DYNCREATE(CNTGraph)
public:
	CLSID const& GetClsid()
	{
		static CLSID const clsid
			= { 0xC9FE01C2, 0x2746, 0x479B, { 0x96, 0xAB, 0xE0, 0xBE, 0x99, 0x31, 0xB0, 0x18 } };
		return clsid;
	}
	virtual BOOL Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle,
						const RECT& rect, CWnd* pParentWnd, UINT nID, 
						CCreateContext* pContext = NULL)
	{ 
		return CreateControl(GetClsid(), lpszWindowName, dwStyle, rect, pParentWnd, nID); 
	}

    BOOL Create(LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, 
				UINT nID, CFile* pPersist = NULL, BOOL bStorage = FALSE,
				BSTR bstrLicKey = NULL)
	{ 
		return CreateControl(GetClsid(), lpszWindowName, dwStyle, rect, pParentWnd, nID,
		pPersist, bStorage, bstrLicKey); 
	}

// 특성입니다.
public:
enum
{
    Solid = 0,
    Dash = 1,
    Dot = 2,
    DashDot = 3,
    DashDotDot = 4,
    Null = 5,
    XYStep = 6,
    YXStep = 7,
    Bars = 8,
    Stick = 9
}LineType;
enum
{
    Nosym = 0,
    Dots = 1,
    Rectangles = 2,
    Diamonds = 3,
    Asterisk = 4,
    DownTriangles = 5,
    UpTriangles = 6,
    LeftTriangles = 7,
    RightTriangles = 8
}SymbolType;
enum
{
    Flat = 0,
    Scope = 1,
    Bitmap = 2
}FrameStyle;
enum
{
    None = 0,
    Track = 1,
    Zoom = 2,
    PanXY = 3,
    PanX = 4,
    PanY = 5
}TrackModeState;


// 작업입니다.
public:

// _DNTGraph

// Functions
//

	void SetRange(double xmin, double xmax, double ymin, double ymax)
	{
		static BYTE parms[] = VTS_R8 VTS_R8 VTS_R8 VTS_R8 ;
		InvokeHelper(0x35, DISPATCH_METHOD, VT_EMPTY, NULL, parms, xmin, xmax, ymin, ymax);
	}
	void AutoRange()
	{
		InvokeHelper(0x36, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void CopyToClipboard()
	{
		InvokeHelper(0x37, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void PrintGraph()
	{
		InvokeHelper(0x38, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void AddElement()
	{
		InvokeHelper(0x39, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void DeleteElement(short ElementID)
	{
		static BYTE parms[] = VTS_I2 ;
		InvokeHelper(0x3a, DISPATCH_METHOD, VT_EMPTY, NULL, parms, ElementID);
	}
	void ClearGraph()
	{
		InvokeHelper(0x3b, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	double get_ElementXValue(long index, short ElementID)
	{
		double result;
		static BYTE parms[] = VTS_I4 VTS_I2 ;
		InvokeHelper(0x44, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, parms, index, ElementID);
		return result;
	}
	void put_ElementXValue(long index, short ElementID, double newValue)
	{
		static BYTE parms[] = VTS_I4 VTS_I2 VTS_R8 ;
		InvokeHelper(0x44, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms, index, ElementID, newValue);
	}
	double get_ElementYValue(long index, short ElementID)
	{
		double result;
		static BYTE parms[] = VTS_I4 VTS_I2 ;
		InvokeHelper(0x45, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, parms, index, ElementID);
		return result;
	}
	void put_ElementYValue(long index, short ElementID, double newValue)
	{
		static BYTE parms[] = VTS_I4 VTS_I2 VTS_R8 ;
		InvokeHelper(0x45, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms, index, ElementID, newValue);
	}
	void PlotXY(double X, double Y, short ElementID)
	{
		static BYTE parms[] = VTS_R8 VTS_R8 VTS_I2 ;
		InvokeHelper(0x3c, DISPATCH_METHOD, VT_EMPTY, NULL, parms, X, Y, ElementID);
	}
	void PlotY(double Y, short ElementID)
	{
		static BYTE parms[] = VTS_R8 VTS_I2 ;
		InvokeHelper(0x3d, DISPATCH_METHOD, VT_EMPTY, NULL, parms, Y, ElementID);
	}
	void ShowProperties()
	{
		InvokeHelper(0x3e, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void SaveAs(LPCTSTR szFilename)
	{
		static BYTE parms[] = VTS_BSTR ;
		InvokeHelper(0x3f, DISPATCH_METHOD, VT_EMPTY, NULL, parms, szFilename);
	}
	void AddAnnotation()
	{
		InvokeHelper(0x40, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void DeleteAnnotation(short annoID)
	{
		static BYTE parms[] = VTS_I2 ;
		InvokeHelper(0x41, DISPATCH_METHOD, VT_EMPTY, NULL, parms, annoID);
	}
	void AddCursor()
	{
		InvokeHelper(0x42, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}
	void DeleteCursor(short cursorID)
	{
		static BYTE parms[] = VTS_I2 ;
		InvokeHelper(0x43, DISPATCH_METHOD, VT_EMPTY, NULL, parms, cursorID);
	}
	void AboutBox()
	{
		InvokeHelper(DISPID_ABOUTBOX, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
	}

// Properties
//

short GetAppearance()
{
	short result;
	GetProperty(DISPID_APPEARANCE, VT_I2, (void*)&result);
	return result;
}
void SetAppearance(short propVal)
{
	SetProperty(DISPID_APPEARANCE, VT_I2, propVal);
}
CString GetCaption()
{
	CString result;
	GetProperty(DISPID_CAPTION, VT_BSTR, (void*)&result);
	return result;
}
void SetCaption(CString propVal)
{
	SetProperty(DISPID_CAPTION, VT_BSTR, propVal);
}
BOOL GetEnabled()
{
	BOOL result;
	GetProperty(DISPID_ENABLED, VT_BOOL, (void*)&result);
	return result;
}
void SetEnabled(BOOL propVal)
{
	SetProperty(DISPID_ENABLED, VT_BOOL, propVal);
}
unsigned long GetAxisColor()
{
	unsigned long result;
	GetProperty(0x1, VT_UI4, (void*)&result);
	return result;
}
void SetAxisColor(unsigned long propVal)
{
	SetProperty(0x1, VT_UI4, propVal);
}
unsigned long GetGridColor()
{
	unsigned long result;
	GetProperty(0x2, VT_UI4, (void*)&result);
	return result;
}
void SetGridColor(unsigned long propVal)
{
	SetProperty(0x2, VT_UI4, propVal);
}
unsigned long GetLabelColor()
{
	unsigned long result;
	GetProperty(0x3, VT_UI4, (void*)&result);
	return result;
}
void SetLabelColor(unsigned long propVal)
{
	SetProperty(0x3, VT_UI4, propVal);
}
unsigned long GetCursorColor()
{
	unsigned long result;
	GetProperty(0x4, VT_UI4, (void*)&result);
	return result;
}
void SetCursorColor(unsigned long propVal)
{
	SetProperty(0x4, VT_UI4, propVal);
}
COleFont GetLabelFont()
{
	LPDISPATCH result;
	GetProperty(0x2b, VT_DISPATCH, (void*)&result);
	return COleFont(result);
}
void SetLabelFont(LPDISPATCH propVal)
{
	SetProperty(0x2b, VT_DISPATCH, propVal);
}
COleFont GetTickFont()
{
	LPDISPATCH result;
	GetProperty(0x2c, VT_DISPATCH, (void*)&result);
	return COleFont(result);
}
void SetTickFont(LPDISPATCH propVal)
{
	SetProperty(0x2c, VT_DISPATCH, propVal);
}
COleFont GetTitleFont()
{
	LPDISPATCH result;
	GetProperty(0x2d, VT_DISPATCH, (void*)&result);
	return COleFont(result);
}
void SetTitleFont(LPDISPATCH propVal)
{
	SetProperty(0x2d, VT_DISPATCH, propVal);
}
COleFont GetIdentFont()
{
	LPDISPATCH result;
	GetProperty(0x2e, VT_DISPATCH, (void*)&result);
	return COleFont(result);
}
void SetIdentFont(LPDISPATCH propVal)
{
	SetProperty(0x2e, VT_DISPATCH, propVal);
}
short GetXGridNumber()
{
	short result;
	GetProperty(0x5, VT_I2, (void*)&result);
	return result;
}
void SetXGridNumber(short propVal)
{
	SetProperty(0x5, VT_I2, propVal);
}
short GetYGridNumber()
{
	short result;
	GetProperty(0x6, VT_I2, (void*)&result);
	return result;
}
void SetYGridNumber(short propVal)
{
	SetProperty(0x6, VT_I2, propVal);
}
BOOL GetShowGrid()
{
	BOOL result;
	GetProperty(0x7, VT_BOOL, (void*)&result);
	return result;
}
void SetShowGrid(BOOL propVal)
{
	SetProperty(0x7, VT_BOOL, propVal);
}
CString GetXLabel()
{
	CString result;
	GetProperty(0x8, VT_BSTR, (void*)&result);
	return result;
}
void SetXLabel(CString propVal)
{
	SetProperty(0x8, VT_BSTR, propVal);
}
CString GetYLabel()
{
	CString result;
	GetProperty(0x9, VT_BSTR, (void*)&result);
	return result;
}
void SetYLabel(CString propVal)
{
	SetProperty(0x9, VT_BSTR, propVal);
}
unsigned long GetElementLineColor()
{
	unsigned long result;
	GetProperty(0xa, VT_UI4, (void*)&result);
	return result;
}
void SetElementLineColor(unsigned long propVal)
{
	SetProperty(0xa, VT_UI4, propVal);
}
unsigned long GetElementPointColor()
{
	unsigned long result;
	GetProperty(0xb, VT_UI4, (void*)&result);
	return result;
}
void SetElementPointColor(unsigned long propVal)
{
	SetProperty(0xb, VT_UI4, propVal);
}
long GetElementLinetype()
{
	long result;
	GetProperty(0xc, VT_I4, (void*)&result);
	return result;
}
void SetElementLinetype(long propVal)
{
	SetProperty(0xc, VT_I4, propVal);
}
long GetElementWidth()
{
	long result;
	GetProperty(0xd, VT_I4, (void*)&result);
	return result;
}
void SetElementWidth(long propVal)
{
	SetProperty(0xd, VT_I4, propVal);
}
long GetElementPointSymbol()
{
	long result;
	GetProperty(0xe, VT_I4, (void*)&result);
	return result;
}
void SetElementPointSymbol(long propVal)
{
	SetProperty(0xe, VT_I4, propVal);
}
BOOL GetElementSolidPoint()
{
	BOOL result;
	GetProperty(0xf, VT_BOOL, (void*)&result);
	return result;
}
void SetElementSolidPoint(BOOL propVal)
{
	SetProperty(0xf, VT_BOOL, propVal);
}
BOOL GetElementShow()
{
	BOOL result;
	GetProperty(0x10, VT_BOOL, (void*)&result);
	return result;
}
void SetElementShow(BOOL propVal)
{
	SetProperty(0x10, VT_BOOL, propVal);
}
long GetTrackMode()
{
	long result;
	GetProperty(0x2f, VT_I4, (void*)&result);
	return result;
}
void SetTrackMode(long propVal)
{
	SetProperty(0x2f, VT_I4, propVal);
}
CString GetElementName()
{
	CString result;
	GetProperty(0x11, VT_BSTR, (void*)&result);
	return result;
}
void SetElementName(CString propVal)
{
	SetProperty(0x11, VT_BSTR, propVal);
}
BOOL GetElementIdentify()
{
	BOOL result;
	GetProperty(0x12, VT_BOOL, (void*)&result);
	return result;
}
void SetElementIdentify(BOOL propVal)
{
	SetProperty(0x12, VT_BOOL, propVal);
}
BOOL GetXLog()
{
	BOOL result;
	GetProperty(0x13, VT_BOOL, (void*)&result);
	return result;
}
void SetXLog(BOOL propVal)
{
	SetProperty(0x13, VT_BOOL, propVal);
}
BOOL GetYLog()
{
	BOOL result;
	GetProperty(0x14, VT_BOOL, (void*)&result);
	return result;
}
void SetYLog(BOOL propVal)
{
	SetProperty(0x14, VT_BOOL, propVal);
}
CPicture GetControlFramePicture()
{
	LPDISPATCH result;
	GetProperty(0x30, VT_DISPATCH, (void*)&result);
	return CPicture(result);
}
void SetControlFramePicture(LPDISPATCH propVal)
{
	SetProperty(0x30, VT_DISPATCH, propVal);
}
CPicture GetPlotAreaPicture()
{
	LPDISPATCH result;
	GetProperty(0x31, VT_DISPATCH, (void*)&result);
	return CPicture(result);
}
void SetPlotAreaPicture(LPDISPATCH propVal)
{
	SetProperty(0x31, VT_DISPATCH, propVal);
}
unsigned long GetControlFrameColor()
{
	unsigned long result;
	GetProperty(0x15, VT_UI4, (void*)&result);
	return result;
}
void SetControlFrameColor(unsigned long propVal)
{
	SetProperty(0x15, VT_UI4, propVal);
}
unsigned long GetPlotAreaColor()
{
	unsigned long result;
	GetProperty(0x16, VT_UI4, (void*)&result);
	return result;
}
void SetPlotAreaColor(unsigned long propVal)
{
	SetProperty(0x16, VT_UI4, propVal);
}
long GetFrameStyle()
{
	long result;
	GetProperty(0x17, VT_I4, (void*)&result);
	return result;
}
void SetFrameStyle(long propVal)
{
	SetProperty(0x17, VT_I4, propVal);
}
CString GetAnnoLabelCaption()
{
	CString result;
	GetProperty(0x18, VT_BSTR, (void*)&result);
	return result;
}
void SetAnnoLabelCaption(CString propVal)
{
	SetProperty(0x18, VT_BSTR, propVal);
}
double GetAnnoLabelX()
{
	double result;
	GetProperty(0x19, VT_R8, (void*)&result);
	return result;
}
void SetAnnoLabelX(double propVal)
{
	SetProperty(0x19, VT_R8, propVal);
}
double GetAnnoLabelY()
{
	double result;
	GetProperty(0x1a, VT_R8, (void*)&result);
	return result;
}
void SetAnnoLabelY(double propVal)
{
	SetProperty(0x1a, VT_R8, propVal);
}
unsigned long GetAnnoLabelColor()
{
	unsigned long result;
	GetProperty(0x1b, VT_UI4, (void*)&result);
	return result;
}
void SetAnnoLabelColor(unsigned long propVal)
{
	SetProperty(0x1b, VT_UI4, propVal);
}
BOOL GetAnnoLabelHorizontal()
{
	BOOL result;
	GetProperty(0x1c, VT_BOOL, (void*)&result);
	return result;
}
void SetAnnoLabelHorizontal(BOOL propVal)
{
	SetProperty(0x1c, VT_BOOL, propVal);
}
short GetAnnotation()
{
	short result;
	GetProperty(0x1d, VT_I2, (void*)&result);
	return result;
}
void SetAnnotation(short propVal)
{
	SetProperty(0x1d, VT_I2, propVal);
}
BOOL GetAnnoVisible()
{
	BOOL result;
	GetProperty(0x1e, VT_BOOL, (void*)&result);
	return result;
}
void SetAnnoVisible(BOOL propVal)
{
	SetProperty(0x1e, VT_BOOL, propVal);
}
short GetAnnoCount()
{
	short result;
	GetProperty(0x32, VT_I2, (void*)&result);
	return result;
}
void SetAnnoCount(short propVal)
{
	SetProperty(0x32, VT_I2, propVal);
}
short GetElement()
{
	short result;
	GetProperty(0x1f, VT_I2, (void*)&result);
	return result;
}
void SetElement(short propVal)
{
	SetProperty(0x1f, VT_I2, propVal);
}
short GetElementCount()
{
	short result;
	GetProperty(0x33, VT_I2, (void*)&result);
	return result;
}
void SetElementCount(short propVal)
{
	SetProperty(0x33, VT_I2, propVal);
}
unsigned long GetAnnoLabelBkColor()
{
	unsigned long result;
	GetProperty(0x20, VT_UI4, (void*)&result);
	return result;
}
void SetAnnoLabelBkColor(unsigned long propVal)
{
	SetProperty(0x20, VT_UI4, propVal);
}
short GetCursorCount()
{
	short result;
	GetProperty(0x34, VT_I2, (void*)&result);
	return result;
}
void SetCursorCount(short propVal)
{
	SetProperty(0x34, VT_I2, propVal);
}
short GetCursor()
{
	short result;
	GetProperty(0x21, VT_I2, (void*)&result);
	return result;
}
void SetCursor(short propVal)
{
	SetProperty(0x21, VT_I2, propVal);
}
double GetCursorX()
{
	double result;
	GetProperty(0x22, VT_R8, (void*)&result);
	return result;
}
void SetCursorX(double propVal)
{
	SetProperty(0x22, VT_R8, propVal);
}
double GetCursorY()
{
	double result;
	GetProperty(0x23, VT_R8, (void*)&result);
	return result;
}
void SetCursorY(double propVal)
{
	SetProperty(0x23, VT_R8, propVal);
}
short GetCursorStyle()
{
	short result;
	GetProperty(0x24, VT_I2, (void*)&result);
	return result;
}
void SetCursorStyle(short propVal)
{
	SetProperty(0x24, VT_I2, propVal);
}
BOOL GetCursorVisible()
{
	BOOL result;
	GetProperty(0x25, VT_BOOL, (void*)&result);
	return result;
}
void SetCursorVisible(BOOL propVal)
{
	SetProperty(0x25, VT_BOOL, propVal);
}
short GetCursorMode()
{
	short result;
	GetProperty(0x26, VT_I2, (void*)&result);
	return result;
}
void SetCursorMode(short propVal)
{
	SetProperty(0x26, VT_I2, propVal);
}
CString GetFormatAxisBottom()
{
	CString result;
	GetProperty(0x27, VT_BSTR, (void*)&result);
	return result;
}
void SetFormatAxisBottom(CString propVal)
{
	SetProperty(0x27, VT_BSTR, propVal);
}
CString GetFormatAxisLeft()
{
	CString result;
	GetProperty(0x28, VT_BSTR, (void*)&result);
	return result;
}
void SetFormatAxisLeft(CString propVal)
{
	SetProperty(0x28, VT_BSTR, propVal);
}
BOOL GetYTime()
{
	BOOL result;
	GetProperty(0x29, VT_BOOL, (void*)&result);
	return result;
}
void SetYTime(BOOL propVal)
{
	SetProperty(0x29, VT_BOOL, propVal);
}
BOOL GetXTime()
{
	BOOL result;
	GetProperty(0x2a, VT_BOOL, (void*)&result);
	return result;
}
void SetXTime(BOOL propVal)
{
	SetProperty(0x2a, VT_BOOL, propVal);
}


};

//#ifndef __BODYINDEX_H__
//#define __BODYINDEX_H__
//#endif 
#pragma once
#include <stdio.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\mat.hpp>
#include <cv.h>
#include <Kinect.h>
#include <Windows.h>
using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class BodyIndexBasic{
public:
	BodyIndexBasic();
	~BodyIndexBasic();	
	HRESULT InitializeDefaultSensor();
	HRESULT InitDepthSensor();
	HRESULT InitBodyIndex();
	HRESULT InitRgbSensor();
	void UpDate();

	void BodyIndexProcess();
	void ColorImageProcess();
	void DepthImageProcess();
	//bounding rectangle of the object, we will use the center of this as its position.
	void DrawRec(Mat thresholdImage, Mat &cameraFeed);
	Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);
	Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);
	void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed);
	string intToString(int number);

private:
	IKinectSensor * kinect;
	IDepthFrameSource * pDepthFrameSource;
	IDepthFrameReader * pDepthFrameReader;

	IBodyIndexFrameSource * pBodyIndexFrameSource;
	IBodyIndexFrameReader * pBodyIndexFrameReader;

	IColorFrameSource * pColorFrameSource;
	IColorFrameReader *pColorFrameReader;
	HRESULT hr;

	int height, ColorHeight;
	int width, ColorWidth;
	Mat img;
	Mat ColorImage;
	Mat thresholdImage;
	Mat NewColorImage;
	Mat DepthImage;
	const static int SENSITIVITY_VALUE = 30;
	Size dsize = Size(520, 424);
	RGBQUAD* m_pColorRGBX;
	UINT Capacity;
	UINT16 * pixData;
	ColorImageFormat imageFormat;
};
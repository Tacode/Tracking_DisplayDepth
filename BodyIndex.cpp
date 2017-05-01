/*BodyIndex,识别人体*/
#include <stdio.h>
#include "BodyIndex.h"
using namespace cv;

string BodyIndexBasic::intToString(int number){
	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void BodyIndexBasic::DrawRec(Mat thresholdImage, Mat &cameraFeed){
	Mat temp;
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	thresholdImage.copyTo(temp);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //寻找轮廓
	if (contours.size() > 0){
		vector < vector<Point> > largestContours;
		largestContours.push_back(contours.at(contours.size() - 1));
		objectBoundingRectangle = boundingRect(largestContours.at(0));//找到外接最小轮廓
		rectangle(cameraFeed, objectBoundingRectangle, Scalar(0, 0, 255), 1, 8, 0);//画出外接轮廓
		int x = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int y = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
		circle(cameraFeed, Point(x, y), 5, Scalar(0, 0, 255), 2);
		//putText(cameraFeed, "Tracking object at (" + intToString(x) + "," + intToString(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
		INT32 pixelIndex = (INT32)(x + ((INT32)y *width));
		//double depth = (pixData[pixelIndex]>>3);
		double depth = (pixData[pixelIndex]);
		double distance = depth / 10;
		if (depth > 0){
			cout << "(" << x << ", " << y << ", ";
			cout << distance << "cm )" << endl;
		}
	}
}

void BodyIndexBasic::searchForMovement(Mat thresholdImage, Mat &cameraFeed){
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	Mat temp;
	bool objectDetected = false;
	int theObject[2] = { 0, 0 };
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contours.size()>0)
		objectDetected = true;
	else 
		objectDetected = false;

	if (objectDetected){
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects positions by changing the 'theObject' array values
		theObject[0] = xpos, theObject[1] = ypos;
	}
	//make some temp x and y variables so we dont have to type out so much
	int x = theObject[0];
	int y = theObject[1];

	//draw some crosshairs around the object
	circle(cameraFeed, Point(x, y), 20, Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x, y - 25), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x, y + 25), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x - 25, y), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x + 25, y), Scalar(0, 0, 255), 2);

	//write the position of the object to the screen
	putText(cameraFeed, "Tracking object at (" + intToString(x) + "," + intToString(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
	cout << "(" + intToString(x) + ", " + intToString(y) + ")" << endl;
}


////////////////////////////////////////////////////////////
HRESULT BodyIndexBasic::InitializeDefaultSensor(){
	HRESULT hr;
	hr = GetDefaultKinectSensor(&kinect);
	setUseOptimized(true);
	if (FAILED(hr)){
		cout << "获取Kinect失败" << endl;
		return E_FAIL;
	}
	if (kinect){
	
		IFrameDescription *pFrameDescription = NULL;
		
		hr = kinect->Open();
		if (FAILED(hr)){
			cout << "打开Kinect失败" << endl;
			return E_FAIL;
		}
		hr=InitBodyIndex();
		hr=InitDepthSensor();
		hr=InitRgbSensor();	

		height = 0;
		width = 0;
		ColorHeight = 0;
		ColorWidth = 0;

		/*BodyIndex图的分辨率--深度图的分辨率*/
		pBodyIndexFrameSource->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Height(&height);
		pFrameDescription->get_Width(&width);
		cout << "width= " << width << " height= " << height << endl;
		pFrameDescription = NULL;
		///*RGB图的分辨率*/
		pColorFrameSource->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Height(&ColorHeight);
		pFrameDescription->get_Width(&ColorWidth);
		cout << "ColorWidth= " << ColorWidth << " ColorHeight= " << ColorHeight << endl;

		m_pColorRGBX = new RGBQUAD[ColorWidth * ColorHeight];
		pixData = new UINT16[height*width];
		Capacity = height*width;

		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyIndexFrameSource);
		SafeRelease(pColorFrameSource);
		SafeRelease(pFrameDescription);
	}
	img.create(height, width, CV_8UC3);
	DepthImage.create(height, width, CV_8UC3);
	ColorImage.create(ColorHeight, ColorWidth,CV_8UC3);
	NewColorImage.create(dsize, CV_8UC3);

	img.setTo(0);
	DepthImage.setTo(0);
	ColorImage.setTo(0);
	NewColorImage.setTo(0);
	return hr;
}


//////////////////////////////////////////////////////////
HRESULT BodyIndexBasic::InitDepthSensor(){
	pDepthFrameSource = NULL;
	pDepthFrameReader = NULL;
	hr = kinect->get_DepthFrameSource(&pDepthFrameSource);
	if (FAILED(hr)){
		cout << "Fail to open the DepthFrameSource";
		return E_FAIL;
	}
	hr = pDepthFrameSource->OpenReader(&pDepthFrameReader);
	if (FAILED(hr)){
		cout << "Fail to open the DepthFrameReader";
		return E_FAIL;
	}
	return S_OK;
}

HRESULT BodyIndexBasic::InitBodyIndex(){
	pBodyIndexFrameSource = NULL;
	pBodyIndexFrameReader = NULL;
	/*初始化BodyIndex*/
	hr = kinect->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
	if (!SUCCEEDED(hr)){
		cout << "获取Body源失败" << endl;
		return E_FAIL;
	}

	hr = pBodyIndexFrameSource->OpenReader(&pBodyIndexFrameReader);
	if (!SUCCEEDED(hr)){
		cout << "打开读取器失败" << endl;
		return E_FAIL;
	}
	return S_OK;
}

HRESULT BodyIndexBasic::InitRgbSensor(){
	pColorFrameSource=NULL;
	pColorFrameReader=NULL;
	cvNamedWindow("Newcolor");
	/*初始化RGB摄像头*/
	hr = kinect->get_ColorFrameSource(&pColorFrameSource);
	if (!SUCCEEDED(hr)){
		cout << "获取Color源失败" << endl;
		return E_FAIL;
	}
	hr = pColorFrameSource->OpenReader(&pColorFrameReader);
	if (!SUCCEEDED(hr)){
		cout << "打开color读取器失败" << endl;
		return E_FAIL;
	}
	return S_OK;
}


/////////////////////////////////////////////////////////
void BodyIndexBasic::UpDate(){
	BodyIndexProcess();
	ColorImageProcess();		
	DepthImageProcess();
}


/////////////////////////////////////////////////////////
void BodyIndexBasic::BodyIndexProcess(){
	img.setTo(0);
	/*ColorImage.setTo(0);*/
	if (!pBodyIndexFrameReader)
	{
		return;
	}
	hr = S_OK;
	IBodyIndexFrame * pBodyIndexFrame = NULL;

	/*BodyIndex图像获取及显示*/
	if (SUCCEEDED(hr)){
		hr = pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	}
	if (SUCCEEDED(hr)){
		//BYTE *bodyIndexArray = new BYTE[height* width];//背景二值图是8为uchar，有人是黑色，没人是白色
		//pBodyIndexFrame->CopyFrameDataToArray(height* width, bodyIndexArray);
		//uchar* Data = (uchar*)img.data;
		//for (int j = 0; j < height * width; ++j){
		//	*Data = bodyIndexArray[j]; ++Data;
		//	*Data = bodyIndexArray[j]; ++Data;
		//	*Data = bodyIndexArray[j]; ++Data;
		//}
		//delete[] bodyIndexArray;
		UINT nBufferSize = 0;
		unsigned char* pBuffer = nullptr;
		pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		
		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)

			{
				unsigned int index = x *width + y;
				if (pBuffer[index] != 255)
				{
					img.at<Vec3b>(x, y) = Vec3b(0, 255, 0);
				}
				else
				{
					img.at<Vec3b>(x, y) = Vec3b(0, 0, 0);
				}
			}
		}
	}
	SafeRelease(pBodyIndexFrame);
	thresholdImage.create(height, width, CV_8UC3);
	cvtColor(img, thresholdImage, COLOR_BGR2GRAY);
	threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//searchForMovement(thresholdImage, img);
	DrawRec(thresholdImage, img);
	cvNamedWindow("BodyIndexImage");
	imshow("BodyIndexImage", img);
	if (waitKey(34) == VK_ESCAPE){
		cvDestroyAllWindows();
		exit(0);
	}

}

void BodyIndexBasic::ColorImageProcess(){
	ColorImage.setTo(0);
	if (!pColorFrameReader){
		return;
	}
	IColorFrame * pColorFrame = NULL;
	hr = S_OK;
	/*Color图像获取及显示*/
	hr = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	//////////////////////////////////////
	UINT nBufferSize_color = 0;
	RGBQUAD *pBuffer_color = NULL;
	//ColorImageFormat imageFormat = ColorImageFormat_None;
	if (SUCCEEDED(hr))
	{
		imageFormat = ColorImageFormat_None;
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}
	/*	if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}*/
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_color, reinterpret_cast<BYTE**>(&pBuffer_color));
			}
			else if (m_pColorRGBX)
			{
				pBuffer_color = m_pColorRGBX;
				nBufferSize_color = ColorWidth * ColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_color, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
			ColorImage = ConvertMat(pBuffer_color, ColorWidth, ColorHeight);
		}
		SafeRelease(pColorFrame);
		resize(ColorImage, NewColorImage, dsize);
		DrawRec(thresholdImage, NewColorImage);

		imshow("Newcolor", NewColorImage);
		if (waitKey(34) == VK_ESCAPE){
			cvDestroyAllWindows();
			exit(0);
		}
	}

}

void BodyIndexBasic::DepthImageProcess(){
	DepthImage.setTo(0);
	hr = S_OK;
	if (!pDepthFrameReader){
		return;
	}
	IDepthFrame* pDepthFrame = NULL;
	UINT nBufferSize_depth = 0;
	UINT16 *pBuffer_depth = NULL;
	hr = pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr)){
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;
		hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
		if (SUCCEEDED(hr)){
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}
		if (SUCCEEDED(hr))
		{
			//cout << "最小限制距离:" << nDepthMinReliableDistance << "最大限制距离:" << nDepthMaxReliableDistance << endl;
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
			DepthImage = ConvertMat(pBuffer_depth, width, height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
		}
		if (SUCCEEDED(hr)){
			//hr = pDepthFrame->CopyFrameDataToArray(Capacity, reinterpret_cast<UINT16 *> (tempImage.data));
			hr = pDepthFrame->CopyFrameDataToArray(Capacity, pixData);

		}
	}
	SafeRelease(pDepthFrame);
	imshow("Depth", DepthImage);
	if (waitKey(34) == VK_ESCAPE){
		cvDestroyAllWindows();
		exit(0);
	}
}



// 转换color图像到cv::Mat  
Mat BodyIndexBasic::ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}

Mat BodyIndexBasic::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat temp(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = temp.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;

		BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;

		++pBuffer;
	}
	return temp;
}



BodyIndexBasic::BodyIndexBasic(){

}

BodyIndexBasic::~BodyIndexBasic(){
	SafeRelease(pBodyIndexFrameReader);
	SafeRelease(pColorFrameReader);
	SafeRelease(pDepthFrameReader);
	if (kinect)
	{
		kinect->Close();
	}
	SafeRelease(kinect);
}

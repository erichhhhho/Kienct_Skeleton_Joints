#include <iostream>
#include <Kinect.h>
#include <opencv2\highgui.hpp>

using   namespace   std;
using   namespace   cv;

int main(void)
{
	IKinectSensor   * mySensor = nullptr;           //第1步打开Sensor
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * mySource = nullptr;       //第2步获取Source
	mySensor->get_ColorFrameSource(&mySource);

	int     height = 0, width = 0;                  //取得宽和高等下用
	IFrameDescription   * myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);

	IColorFrameReader   * myReader = nullptr;       //第3步打开Reader
	mySource->OpenReader(&myReader);

	Mat img(height, width, CV_8UC4);
	IColorFrame     * myFrame = nullptr;
	while (1)
	{
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK) //第4步获取Frame
		{
			UINT    size = 0;
			myFrame->CopyConvertedFrameDataToArray(width * height * 4, (BYTE *)img.data, ColorImageFormat_Bgra);
			imshow("TEST", img);
			myFrame->Release();
		}
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myReader->Release();        //记得要释放
	myDescription->Release();
	mySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}
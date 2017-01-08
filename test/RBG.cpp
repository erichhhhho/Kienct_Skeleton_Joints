#include <iostream>
#include <Kinect.h>
#include <opencv2\highgui.hpp>

using   namespace   std;
using   namespace   cv;

int main(void)
{
	IKinectSensor   * mySensor = nullptr;           //��1����Sensor
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * mySource = nullptr;       //��2����ȡSource
	mySensor->get_ColorFrameSource(&mySource);

	int     height = 0, width = 0;                  //ȡ�ÿ�͸ߵ�����
	IFrameDescription   * myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);

	IColorFrameReader   * myReader = nullptr;       //��3����Reader
	mySource->OpenReader(&myReader);

	Mat img(height, width, CV_8UC4);
	IColorFrame     * myFrame = nullptr;
	while (1)
	{
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK) //��4����ȡFrame
		{
			UINT    size = 0;
			myFrame->CopyConvertedFrameDataToArray(width * height * 4, (BYTE *)img.data, ColorImageFormat_Bgra);
			imshow("TEST", img);
			myFrame->Release();
		}
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myReader->Release();        //�ǵ�Ҫ�ͷ�
	myDescription->Release();
	mySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}
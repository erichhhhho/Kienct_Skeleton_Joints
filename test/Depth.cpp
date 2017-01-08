#include <Kinect.h>
#include <iostream>
#include <opencv2\highgui.hpp>

using   namespace   std;
using   namespace   cv;
int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);  //获取感应器
	mySensor->Open();           //打开感应器

	IDepthFrameSource   * mySource = nullptr;   //取得深度数据
	mySensor->get_DepthFrameSource(&mySource);

	int height = 0, width = 0;
	IFrameDescription   * myDescription = nullptr;  //取得深度数据的分辨率
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();

	IDepthFrameReader   * myReader = nullptr;
	mySource->OpenReader(&myReader);    //打开深度数据的Reader

	IDepthFrame * myFrame = nullptr;
	Mat temp(height, width, CV_16UC1);    //建立图像矩阵
	Mat img(height, width, CV_8UC1);
	while (1)
	{
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK) //通过Reader尝试获取最新的一帧深度数据，放入深度帧中,并判断是否成功获取
		{
			myFrame->CopyFrameDataToArray(height * width, (UINT16 *)temp.data); //先把数据存入16位的图像矩阵中
			temp.convertTo(img, CV_8UC1, 255.0 / 4500);   //再把16位转换为8位
			imshow("TEST", img);
			myFrame->Release();
		}
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myReader->Release();        //释放不用的变量并且关闭感应器
	mySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}
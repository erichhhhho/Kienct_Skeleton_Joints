#include <Kinect.h>
#include <iostream>
#include <opencv2\highgui.hpp>

using   namespace   std;
using   namespace   cv;
int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);  //��ȡ��Ӧ��
	mySensor->Open();           //�򿪸�Ӧ��

	IDepthFrameSource   * mySource = nullptr;   //ȡ���������
	mySensor->get_DepthFrameSource(&mySource);

	int height = 0, width = 0;
	IFrameDescription   * myDescription = nullptr;  //ȡ��������ݵķֱ���
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();

	IDepthFrameReader   * myReader = nullptr;
	mySource->OpenReader(&myReader);    //��������ݵ�Reader

	IDepthFrame * myFrame = nullptr;
	Mat temp(height, width, CV_16UC1);    //����ͼ�����
	Mat img(height, width, CV_8UC1);
	while (1)
	{
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK) //ͨ��Reader���Ի�ȡ���µ�һ֡������ݣ��������֡��,���ж��Ƿ�ɹ���ȡ
		{
			myFrame->CopyFrameDataToArray(height * width, (UINT16 *)temp.data); //�Ȱ����ݴ���16λ��ͼ�������
			temp.convertTo(img, CV_8UC1, 255.0 / 4500);   //�ٰ�16λת��Ϊ8λ
			imshow("TEST", img);
			myFrame->Release();
		}
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myReader->Release();        //�ͷŲ��õı������ҹرո�Ӧ��
	mySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}
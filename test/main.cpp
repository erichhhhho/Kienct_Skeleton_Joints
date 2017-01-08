#include <iostream>
#include <opencv2\highgui.hpp>
#include <string>
#include <Kinect.h>
#include <fstream>
#include <streambuf>

using   namespace   std;
using   namespace   cv;

const   string  get_name(int n);    //此函数判断出关节点的名字
int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	int myBodyCount = 0;
	IBodyFrameSource    * myBodySource = nullptr;
	IBodyFrameReader    * myBodyReader = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);

	IDepthFrameSource   * myDepthSource = nullptr;
	IDepthFrameReader   * myDepthReader = nullptr;
	mySensor->get_DepthFrameSource(&myDepthSource);
	myDepthSource->OpenReader(&myDepthReader);

	int height = 0, width = 0;
	IFrameDescription   * myDescription = nullptr;;
	myDepthSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);   //以上为准备好深度数据和骨骼数据的Reader

	IBodyFrame  * myBodyFrame = nullptr;
	IDepthFrame * myDepthFrame = nullptr;
	Mat img16(height, width, CV_16UC1); //为显示深度图像做准备
	Mat img8(height, width, CV_8UC1);

	ofstream output;
	output.open("kinect.csv", ios::out | ios::trunc);
	output << "时间" << "," << "JointType_SpineBase" << "," << "," << "," \
		            << "JointType_SpineMid" << "," << "," << "," \
					<< "JointType_Neck" << "," << "," << "," \
					<< "JointType_Head" << "," << "," << "," \
					<< "JointType_ShoulderLeft " << "," << "," << "," \
					<< "JointType_ElbowLeft  " << "," << "," << "," \
					<< "JointType_WristLeft " << "," << "," << "," \
					<< "JointType_HandLeft  " << "," << "," << "," \
					<< "JointType_ShoulderRight " << "," << "," << "," \
					<< "JointType_ElbowRight  " << "," << "," << "," \
					<< "JointType_WristRight " << "," << "," << "," \
					<< "JointType_HandRight  " << "," << "," << "," \
					<< "JointType_HipLeft " << "," << "," << "," \
					<< "JointType_KneeLeft  " << "," << "," << "," \
					<< "JointType_AnkleLeft " << "," << "," << "," \
					<< "JointType_FootLeft  " << "," << "," << "," \
					<< "JointType_HipRight " << "," << "," << "," \
					<< "JointType_KneeRight  " << "," << "," << "," \
					<< "JointType_AnkleRight " << "," << "," << "," \
					<< "JointType_FootRight  " << "," << "," << "," \
					<< "JointType_SpineShoulder " << "," << "," << "," \
					<< "JointType_HandTipLeft  " << "," << "," << "," \
					<< "JointType_ThumbLeft " << "," << "," << "," \
					<< "JointType_HandTipRight  " << "," << "," << "," \
					<< "JointType_ThumbRight " << "," << "," << "," \
					<< endl;
	int count = 0;
	while (1)
	{
		
		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->CopyFrameDataToArray(width * height, (UINT16 *)img16.data);
		img16.convertTo(img8, CV_8UC1, 255.0 / 4500);
		imshow("Depth Img", img8);  //深度图像的转化及显示

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

		int myBodyCount = 0;
		IBody   ** bodyArr = nullptr;
		myBodySource->get_BodyCount(&myBodyCount);
		bodyArr = new IBody *[myBodyCount];
		for (int i = 0; i < myBodyCount; i++)   //bodyArr的初始化
			bodyArr[i] = nullptr;

		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);
		

		for (int i = 0; i < myBodyCount; i++)   //遍历6个人(可能用不完)
		{
			BOOLEAN     result = false;
			if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //判断此人是否被侦测到
			{
				//output << "Body " << i << " tracked!" << endl;
				//默认单人实验

		    	//int count = 0;
				Joint   jointArr[JointType_Count];
				bodyArr[i]->GetJoints(JointType_Count, jointArr);    //获取此人的关节数据
				
				output << count << ",";
				count++;
				for (int j = 0; j < JointType_Count; j++)
				{
					if (jointArr[j].TrackingState != TrackingState_Tracked) //将确定侦测到的关节显示出来
					output << "NULL"<< "," << "NULL" << "," << "NULL" << ",";
					else
					output << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << ",";
					//string  rt = get_name(jointArr[j].JointType);   //获取关节的名字


					//if (rt != "NULL")   //输出关节信息
					//{
					//	count++;
					//	output << "   " << rt << " tracked" << endl;

					//	if (rt == "SpineBase")
					//		output << "       SpineBase at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "SpineMid")
					//		output << "       SpineMid at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "ElbowLeft")
					//		output << "       ElbowLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "WristLeft")
					//		output << "       WristLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "ElbowRight")
					//		output << "       ElbowRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "WristRight")
					//		output << "       WristRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "HipLeft")
					//		output << "       HipLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "KneeLeft")
					//		output << "       KneeLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "AnkleLeft")
					//		output << "       AnkleLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "FootLeft")
					//		output << "       FootLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "HipRight")
					//		output << "       HipRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "KneeRight")
					//		output << "       KneeRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "AnkleRight")
					//		output << "       AnkleRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "FootRight")
					//		output << "       FootRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "SpineShoulder")
					//		output << "      SpineShoulder at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "HandTipLeft")
					//		output << "       HandTipLeft at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	if (rt == "HandTipRight")
					//		output << "       HandTipRight at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;


					//	//24
					//	if (rt == "Right thumb")
					//		output << "       Right thumb at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	//2
					//	if (rt == "Neck")
					//		output << "       Neck at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;	
					//	//3
					//	if (rt == "Head")
					//		output << "       Head at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//    //4
					//	if (rt == "Left shoulder")
					//		output << "       Left Shoulder at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	//8
					//	if (rt == "Right shoulder")
					//		output << "       Right Shouler at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	//7
					//	if (rt == "Left hand")
					//		output << "       Left hand at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	//11
					//	if (rt == "Right hand")
					//		output << "       Right hand at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					//	//22
					//	if (rt == "Left thumb")
					//		output << "       Left thumb at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;


					//}
				}
				output << endl;
				
				//output << count << " joints tracked" << endl << endl;
			}
		}
		myDepthFrame->Release();
		myBodyFrame->Release();
		delete[] bodyArr;

		if (waitKey(30) == VK_ESCAPE)
			break;
		Sleep(100);    //为避免数据刷太快，每秒钟更新一次
		
	}

	output.close();

	myBodyReader->Release();
	myDepthReader->Release();
	myBodySource->Release();
	myDepthSource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

const   string  get_name(int n)
{
	switch (n)
	{
	case    0:return "SpineBase"; break;
	case    1:return "SpineMid"; break;
	case    5:return "ElbowLeft"; break;
	case    6:return "WristLeft"; break;
	case    9:return "ElbowRight"; break;
	case    10:return "WristRight"; break;
	case    12:return "HipLeft"; break;
	case    13:return "KneeLeft"; break;
	case    14:return "AnkleLeft"; break;
	case    15:return "FootLeft"; break;
	case    16:return "HipRight"; break;
	case    17:return "KneeRight"; break;
	case    18:return "AnkleRight"; break;
	case    19:return "FootRight"; break;
	case    20:return "SpineShoulder"; break;
	case    21:return "HandTipLeft"; break;
	case    23:return "HandTipRight"; break;

	case    2:return    "Neck"; break;
	case    3:return    "Head"; break;
	case    4:return    "Left shoulder"; break;
	case    8:return    "Right shoulder"; break;
	case    7:return    "Left hand"; break;
	case    11:return   "Right hand"; break;
	case    22:return   "Left thumb"; break;
	case    24:return   "Right thumb"; break;
	default:return "NULL";
	}
}



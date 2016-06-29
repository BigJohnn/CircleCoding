//Copyright(c) 2014, Hou Peihong
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met :
//
//1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation 
//and / or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software
//without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//�����㷨��ʵ�֣���[����, ������, ֣����,��. ������Ӱ�����б�ǵ���Զ����[J]. Ӧ�ÿ�ѧѧ��, 2007, 25(3):288-294.]

//#include "opencv2\opencv.hpp"
#include "opencv\highgui.h"
#include "opencv\cxcore.h"
#include "opencv\cv.h"
#include "CvvImage.h"
#include <math.h>
#include <iostream>
#include <fstream>
using namespace std;
int lowThreshold;
int const max_lowThreshold = 1;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Circle Coding";
//int bState = 1;

//��Բ���ģ�������뾶����ת�ǡ�
struct Point2d
{
	double x;
	double y;

	double r1;
	double r2;

	double angle;
};

vector<Point2d> list;
vector<Point2d> list1;

vector<Point2d> listu;
vector<Point2d> listd;

void ScaleConstraint(IplImage *dst);
void ShapeConstraint(IplImage *src);
void GrayConstraint();
void PositionConstraint();
void SignCode(IplImage* src);
int  FindNearestDot(int x, int y);
void ReadCalData(vector<Point2d>&list1, char *filename);

int main()
{
	IplImage* src = cvLoadImage("0L.bmp");
	if (!src)
		return -1;

	IplImage* gray, *dst;
	gray = cvCreateImage(cvGetSize(src), 8, 1);
	dst  = cvCreateImage(cvGetSize(src), 8, 1);

	cvCvtColor(src, gray, CV_BGR2GRAY);

	/*******�˴�������������ѡ���ǹؼ�******/
	cvSmooth(gray, gray,2,13,0);
	/*******�˴�������������ѡ���ǹؼ�******/

	cvThreshold(gray, dst, 200, 255, CV_THRESH_OTSU);   // ������ж�ֵ��
//	cvCreateTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, on_trackbar);

	//������ȥ�벢����һ��
	cvErode(src, src, NULL, 1);
	cvDilate(src,src,NULL,1);
	cvDilate(src, src, NULL, 1);

	ScaleConstraint(dst);

	ShapeConstraint(dst);

	GrayConstraint();

	PositionConstraint();

	SignCode(dst);

	cvNamedWindow(window_name);
	cvShowImage(window_name, dst);
	cvWaitKey(0);
	cvDestroyWindow(window_name);
	cvReleaseImage(&dst);
	return 0;
}

//1st step
void ScaleConstraint(IplImage *dst)
{
	CvMemStorage* stor;
	CvSeq* cont;
	CvSeq* result;

	IplImage *CenterCircle = cvCreateImage(cvGetSize(dst), 8, 1);
	IplImage *andst = cvCreateImage(cvGetSize(dst), 8, 1);
	cvZero(CenterCircle);
	stor = cvCreateMemStorage(0);
	cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);

	cvCanny(dst, dst, lowThreshold, lowThreshold*ratio, kernel_size);

	cvFindContours(dst, stor, &cont, sizeof(CvContour), CV_FILLED, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

	for (; cont; cont = cont->h_next)
	{
		result = cvApproxPoly(cont, sizeof(CvContour), stor,
			CV_POLY_APPROX_DP, 0.9, 0);              //ָ�����ȱƽ����������

		//�߶ȹ���
		if (fabs(cvContourArea(cont, CV_WHOLE_SEQ))>1500 &&
			4 * CV_PI*fabs(cvContourArea(result, CV_WHOLE_SEQ)) /
			cvArcLength(result, CV_WHOLE_SEQ, -1) / cvArcLength(result, CV_WHOLE_SEQ, -1)>0.3)
		{
			cvDrawContours(CenterCircle, cont, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8, cvPoint(0, 0));
		}
	}
//	cvAnd(dst, CenterCircle, andst,NULL);
//	if (bState == 1)
//	{
//		ShapeConstraint(CenterCircle);
//		dst = cvCloneImage(CenterCircle);
//	}
//	if (bState == 2)
//	{
//	ShapeConstraint(CenterCircle);
	dst = cvCloneImage(CenterCircle);
//	}
	
	cvClearMemStorage(stor);
}

//2nd step
void ShapeConstraint(IplImage* src)
{
	CvMemStorage* stor;
	CvSeq* cont;
	CvPoint* PointArray;
	CvPoint2D32f* PointArray2D32f;
	CvBox2D32f* box;
	Point2d pt;
	CvPoint center;
	IplImage *dst = cvCloneImage(src);
//	IplImage *srccopy = cvCloneImage(src);
	stor = cvCreateMemStorage(0);
	cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);

//	cvDilate(src, dst, NULL, 4);

	cvFindContours(src, stor, &cont, sizeof(CvContour),
		CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

//	cvZero(dst);
	for (; cont; cont = cont->h_next)
	{
		int i; // Indicator of cycle.
		int count = cont->total; // This is number point in contour
		CvSize size;

		//Important!!!!!
		if (count > 40)
		//	if (count<6)
			continue;

		PointArray = (CvPoint*)malloc(count*sizeof(CvPoint));
		PointArray2D32f = (CvPoint2D32f*)malloc(count*sizeof(CvPoint2D32f));

		box = (CvBox2D32f*)malloc(sizeof(CvBox2D32f));

		cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);

		for (i = 0; i < count; i++)
		{
			PointArray2D32f[i].x = (float)PointArray[i].x;
			PointArray2D32f[i].y = (float)PointArray[i].y;
		}
		//��ϵ�ǰ����.
		cvFitEllipse(PointArray2D32f, count, box);

		center.x = cvRound(box->center.x);
		center.y = cvRound(box->center.y);
		size.width = cvRound(box->size.width*0.5);
		size.height = cvRound(box->size.height*0.5);
		box->angle =box->angle;              //ע��

		pt.x = box->center.x;
		pt.y = box->center.y;
		pt.r1 = box->size.width*0.5;
		pt.r2 = box->size.height*0.5;
		pt.angle = box->angle;

		cvEllipse(dst, center, size,
			box->angle, 0, 360,
			CV_RGB(255, 255, 255), 1, 8, 0);


//		if (bState==1)
//		listd.push_back(pt);
//		if (bState==2)
		listu.push_back(pt);
	}

	cvAbsDiff(dst, src, src);
}

//3rd step
void GrayConstraint()
{

}

//4th step
vector<int> imin;
void PositionConstraint()
{
	for (int i = 0; i<listd.size(); i++)
	{
		imin.push_back(FindNearestDot(listd[i].x, listd[i].y));
	}

	char* filename;
	ofstream fs;
	filename = "pcd_out_order.txt";
	fs.open(filename);
	for (int j = 0; j<listd.size(); j++)
	{
		fs << j << "\t" << listu[imin[j]].x << "\t" << listu[imin[j]].y << "\t" <<
			listu[imin[j]].r1 <<"\t" << listu[imin[j]].r2 << "\t" << listd[j].angle<<"\n";
	}
	fs.close();
}

int code[15];
///<summary>Ultimate Processing</summary>
void SignCode(IplImage* src)
{
	//size.cx = cimg.Width();
	//size.cy = cimg.Height();

	CvFont font;
	int line_type = CV_AA;
	char b[3];
	int s = 0;

	IplImage *srcImg = cvCloneImage(src);
	//�������������е�list1
	ReadCalData(list1, "pcd_out_order.txt");
	//ReadCalData(list1, "circle_data.txt");

	int ptcode[17][60];
	float U1[17];
	float U2[17];
	float mx[17], my[17];
	int kk;

	//�������ϵ�ÿ�����ص���б��
	for (int i = 0; i<17; i++)
	{
		for (int j = 0; j<60; j++)
		{
			float theta = (float)j / (float)60 * 2 * CV_PI;
			float angle = list1[i].angle*CV_PI / 180;

			U1[i] = 3.4*list1[i].r1*cos(theta);
			U2[i] = 3.4*list1[i].r2*sin(theta);

			mx[i] = (U1[i] * cos(angle) - U2[i] * sin(angle) + list1[i].x);
			my[i] = (list1[i].y - U1[i] * sin(angle) - U2[i] * cos(angle));
			
			kk = (((srcImg->imageData) + srcImg->widthStep*(int)(my[i])))[(int)mx[i]]/ 255;
			ptcode[i][j] = kk;

			cvCircle(src, cvPoint(mx[i], my[i]), 0.5, CV_RGB(255, 0, 0), 2, 8, 0);
		}
	}

	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0.0, 1, line_type);

	FILE *fp;
	fp = fopen("result_9R.txt", "w");
//	char* filename;
//	ofstream fp;
//	filename = "result_9R.txt";
//	fp.open(filename);

	int ck[17];

	//ÿ��24����Ϊһ��������λ����15λ
	int tempcode[15];
	for (int i = 0; i<17; i++)
	{	
		for (int j = 0; j<60; j += 4)
		{
			if ((ptcode[i][j] + ptcode[i][j + 1] + ptcode[i][j + 2] + ptcode[i][j + 3]) / 4.0>0.5)
				tempcode[j / 4] = 1;
			else tempcode[j / 4] = 0;
		}

		//ȡ���������Ӧ����Сʮ��������Ϊ����Ԫ��ID��code[i])
		for (int j = 0; j<15; j++)
		{
			if (tempcode[j] == 0 && tempcode[(j + 1) % 15] == 1 && tempcode[(j + 2) % 15] == 1
				&& tempcode[(j + 3) % 15] == 1 && tempcode[(j + 4) % 15] == 0)
				for (int n = 0; n<15; n++)
				{
				code[n] = tempcode[(j + n) % 15];
				}
			_gcvt(tempcode[j], 3, b);
			cvPutText(src, b, cvPoint(list1[i].x - 58 + j * 6, list1[i].y + 40), &font, CV_RGB(0, 255, 0));
		}

		for (int j = 0; j<15; j++)
		{
			_gcvt(code[j], 3, b);
//			cvPutText(src, b, cvPoint(list1[i].x - 58 + j * 6, list1[i].y + 60), &font, CV_RGB(0, 255, 255));
		}

		if (code[0] == 0 && code[1] == 0)
		{
			cvPutText(src, "CDUnit!", cvPoint(list1[i].x, list1[i].y), &font, CV_RGB(255, 0, 255));
		}

		s++;
		_gcvt(s, 3, b);
		cvPutText(src, b, cvPoint(list1[i].x, list1[i].y), &font, CV_RGB(0, 255, 0));

		
		ck[i] = pow(2, 9)*code[5] + pow(2, 8)*code[6] + pow(2, 7)*code[7] + pow(2, 6)*code[8] +
			pow(2, 5)*code[9] + pow(2, 4)*code[10] + pow(2, 3)*code[11] + pow(2, 2)*code[12] +
			pow(2, 1)*code[13] + pow(2, 0)*code[14];

		fprintf(fp, "%d\t%f\t%f\t%d\n", s, list1[i].x, list1[i].y, ck[i]);
		//fp << s << "\t" << list1[i].x << "\t" << list1[i].y << "\t" << ck[i] << "\n";
		for (int j = 0; j<15; j++)
		{
			code[j] = 0;
		}
		
	}
	//fp.close();
}

///<summary>
///Ѱ������㣬��������ֵ
///</summary>
int FindNearestDot(int x, int y)
{
	double min_d = (listu[0].x - x)*(listu[0].x - x) +
		(listu[0].y - y)*(listu[0].y - y);

	int index = 0;
	for (int i = 0; i<listu.size(); i++)
	{
		if ((listu[i].x - x)*(listu[i].x - x) +
			(listu[i].y - y)*(listu[i].y - y)<min_d)
		{
			min_d = (listu[i].x - x)*(listu[i].x - x) +
				(listu[i].y - y)*(listu[i].y - y);
			index = i;
		}
	}
	return index;
}

void ReadCalData(vector<Point2d>&list1, char *filename)
{
	int i;
	double px, py, wx, wy, angle;
	//char* filename;
	//filename = "circle_data.txt";              //�����ļ�"asc.txt"
	ifstream fm;
	fm.open(filename);
	if (fm)
	{
		while (fm.good())
		{
			fm >> i >> px >> py >> wx >> wy >> angle;                   //ÿ�����ݵ���ֵ
			Point2d pt4;
			pt4.x = px;
			pt4.y = py;
			pt4.r1 = wx;
			pt4.r2 = wy;
			pt4.angle = angle;
			list1.push_back(pt4);
		}
	}
	fm.close();
}


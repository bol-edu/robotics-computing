//#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "Methods.h"
#include "Mat.h"
#include <vector>
//#include "Dataset_Handler.h"
//#include "test.h"



//using namespace cv;
using namespace std;

//void tutorial(Methods method, test test1);
//void tutorial(Methods method);

int main()
{


	//Methods method;


	

	//cv::Mat mask = cv::Mat::zeros(handler.imheight, handler.imwidth, CV_8U);
	//mask(cv::Rect(96, 0, handler.imwidth - 96, handler.imheight)) = 255;

	cout<<"start"<<endl;

	int i = 1;

	Mat mask;
	Mat image_test;
	
//#pragma warning(disable:4996)
	FILE* fpmr;
	fpmr = fopen("../../../../workspace/orb_own/input_own/mask.txt", "r");
	int vnum1, vnum2;
	int data;
	int flag, dim, row, col, size0, size1;
	unsigned long long p1, p2;
	unsigned char datause;

	fscanf(fpmr, "%d %d %lld %lld %d %d %d %d", &flag, &dim, &p1, &p2, &row, &col, &size0, &size1);


	mask.flags = flag;
	mask.dims = dim;
	mask.step[0] = p1;
	mask.step[1] = p2;
	mask.rows = row;
	mask.cols = col;
	//mask.size[0] = size0;
	//mask.size[1] = size1;
	static unsigned char ch[377 * 1241];
	cout << "r: " << mask.rows << " ,c: " << mask.cols << endl;

	mask.data = ch;
	int imcol = mask.cols;
	for (int j = 0; j < mask.rows; j++)
	{
		for (int k = 0; k < mask.cols; k++)
		{
			fscanf(fpmr, "%d %d", &vnum1, &vnum2);
			fscanf(fpmr, "%d", &data);

			datause = static_cast<unsigned char>(data);
			int num = k + (j * imcol);
			//if (j >= 375)
			//{cout<<j<<", "<<k<<" " << "datause: " << data << endl; }
			//if(j == 375 && k == 1240)

			mask.data[num] = datause;

		}
	}

	fclose(fpmr);

	cout<<"flag"<<endl;

	FILE* fpt;
	fpt = fopen( "../../../../workspace/orb_own/input_own/mask_check.txt", "w");

	fprintf(fpt, "%d\t%d\t%d :\n",
		i,
		i,
		i);
	fprintf(fpt, "flags: %d\tdims: %d\tstep: %lld\t",
		mask.flags,
		mask.dims,
		mask.step[0]);
	fprintf(fpt, "%lld\trows: %d\tcols: %d\size0: %d\size1: %d \n ", mask.step[1],mask.rows, mask.cols, mask.size[0], mask.size[1]);

	//int imcol = mask.cols;
	for (int j = 0; j < mask.rows; j++)
	{
		for (int k = 0; k < mask.cols; k++)
		{
			fprintf(fpt, " ( ");
			fprintf(fpt, "%d, ", j);
			fprintf(fpt, "%d ): ", k);
			fprintf(fpt, " data: %d ", mask.at<unsigned char>(j, k));
			fprintf(fpt, " %d ", mask.data[k + (j * imcol)]);
			fprintf(fpt, "\n");
		}
	}

	fclose(fpt);


/// /////////////////  input matdata  /////////////////////////////


	FILE* fpmr1;
	fpmr1 = fopen("../../../../workspace/orb_own/input_own/matdata.txt", "r");
	//int vnum1, vnum2;
	int datam;
	//int flag, dim, row, col;
	//unsigned long long p1, p2;
	//unsigned char datause;

	fscanf(fpmr1, "%d %d %lld %lld %d %d", &flag, &dim, &p1, &p2, &row, &col);

	//image_test = image_left;

	image_test.flags = flag;
	image_test.dims = dim;
	image_test.step[0] = p1;
	image_test.step[1] = p2;
	image_test.rows = row;
	image_test.cols = col;
	//cout<<"step: "<<image_test.step[0]<<" "<<image_test.step[1]<<endl;

	unsigned char ch1[1241 * 376];

	image_test.data = ch1;
	int imcol1 = image_test.cols;
	for (int j = 0; j < image_test.rows; j++)
	{
		for (int k = 0; k < image_test.cols; k++)
		{
			fscanf(fpmr1, "%d %d", &vnum1, &vnum2);
			fscanf(fpmr1, "%d", &datam);

			datause = static_cast<unsigned char>(datam);
			int num = k + (j * imcol1);
			image_test.data[num] = datause;

			//if (j >= 375)
			//	{cout<<j<<", "<<k<<" " << "datause: " << static_cast<int>(datause )<< endl; }


		}
	}

	fclose(fpmr1);



	FILE* fpt1;
	fpt1 = fopen( "../../../../workspace/orb_own/input_own/matdata_check.txt", "w");



	fprintf(fpt1, "%d\t%d\t%d :\n",
		i,
		i,
		i);
	fprintf(fpt1, "flags: %d\tdims: %d\tstep: %lld\t",
		image_test.flags,
		image_test.dims,
		image_test.step[0]);
	//fprintf_s(fpt1, "%lld\trows: %d\tcols: %d \n ", image_test.step[1], image_test.rows, image_test.cols);
	fprintf(fpt1, "%lld\trows: %d\tcols: %d\size0: %d\size1: %d \n ", image_test.step[1], image_test.rows, image_test.cols, image_test.size[0], image_test.size[1]);


	//int imcol1 = image_test.cols;
	for (int j = 0; j < image_test.rows; j++)
	{
		for (int k = 0; k < image_test.cols; k++)
		{
			fprintf(fpt1, " ( ");
			fprintf(fpt1, "%d, ", j);
			fprintf(fpt1, "%d ): ", k);
			fprintf(fpt1, " data: %d ", image_test.at<unsigned char>(j, k));
			fprintf(fpt1, " %d ", image_test.data[k + (j * imcol1)]);
			fprintf(fpt1, "\n");
		}
	}

	fclose(fpt1);




	//vector<Mat> trajectory_nolidar_bm =
	//	method.visual_odometry(mask);

	
//cout<<"finish"<<endl;
	


	vector<KeyPoint> kp0;

	//cout << "image_test: " << image_test.step[1] << endl;

	Mat des0 = extract_features(image_test, mask, &kp0);
	//cout << "des0: " << des0.dims << endl;
	cout << "end " << endl;

	FILE* fp1;
	fp1 = fopen("../../../../workspace/orb_own/output_own/out_keypoint.txt", "w");


	fprintf(fp1, "%d\t%d\t%d :\n",
		i,
		i,
		i);
	for (int v = 0; v < kp0.size(); v++)
	{
		fprintf(fp1, "%d: ( ", v);
		fprintf(fp1, "%f, ", kp0.at(v).pt.x);
		fprintf(fp1, "%f )", kp0.at(v).pt.y);
		fprintf(fp1, " size: %f ", kp0.at(v).size);
		fprintf(fp1, " angle: %f ", kp0.at(v).angle);
		fprintf(fp1, " response: %f ", kp0.at(v).response);
		fprintf(fp1, " octave: %d ", kp0.at(v).octave);
		fprintf(fp1, " class_id: %d ", kp0.at(v).class_id);
		fprintf(fp1, "\n");
	}

	fclose(fp1);

	cout << "kp end " << endl;
}


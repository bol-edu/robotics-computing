#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "Methods.h"
//#include "Mat.h"
//#include "KeyPoint.h"
//#include <vector>

using namespace std;


int main()
{
    cout << "Hello world!" << endl;
    int i = 1;
    int o;
    o = i + 5;
    i++;
    //int kp[500];

    //int ans = extract_features( i, o,  kp);
    //cout<<"ans:"<<ans<<endl;

    //Mat mask;
    //Mat image_test;


//#pragma warning(disable:4996)
	FILE* fpmr;
	fpmr = fopen("../../../input_own/mask.txt", "r");
	int vnum1, vnum2;
	int data;
	int flag, dim, row, col, size0, size1;
	unsigned long long p1, p2;
	unsigned char datause;

	fscanf(fpmr, "%d %d %lld %lld %d %d %d %d", &flag, &dim, &p1, &p2, &row, &col, &size0, &size1);


	//mask.flags = flag;
	//mask.dims = dim;
	//mask.step[0] = p1;
	//mask.step[1] = p2;
	//mask.rows = row;
	//mask.cols = col;
	//mask.size[0] = size0;
	//mask.size[1] = size1;
	unsigned char ch[377 * 1241];
	cout << "r: " << row << " ,c: " << col << endl;

	//mask.data = ch;
	int imcol = col;
	for (int j = 0; j < row; j++)
	{
		for (int k = 0; k < col; k++)
		{
			fscanf(fpmr, "%d %d", &vnum1, &vnum2);
			fscanf(fpmr, "%d", &data);

			datause = static_cast<unsigned char>(data);
			int num = k + (j * imcol);
			//if (j >= 375)
			//{cout<<j<<", "<<k<<" " << "datause: " << data << endl; }

			ch[num] = datause;

		}
	}

	fclose(fpmr);



	FILE* fpt;
	fpt = fopen( "../../../input_own/mask_check.txt", "w");

	fprintf(fpt, "%d\t%d\t%d :\n",
		i,
		i,
		i);
	fprintf(fpt, "flags: %d\tdims: %d\tstep: %lld\t",
		flag,
		dim,
		p1);
	fprintf(fpt, "%lld\trows: %d\tcols: %d\size0: %d\size1: %d \n ", p2,row, col, size0, size1);

	//int imcol = mask.cols;
	for (int j = 0; j < row; j++)
	{
		for (int k = 0; k < col; k++)
		{
			fprintf(fpt, " ( ");
			fprintf(fpt, "%d, ", j);
			fprintf(fpt, "%d ): ", k);
			//fprintf(fpt, " data: %d ", mask.at<unsigned char>(j, k));
			fprintf(fpt, " %d ", ch[k + (j * imcol)]);
			//if(j== 0 && j <10)
			//cout<<"data: "<<static_cast<int>(ch[i])<<endl;
			fprintf(fpt, "\n");
		}
	}

	fclose(fpt);


/// /////////////////  input matdata  /////////////////////////////


	FILE* fpmr1;
	fpmr1 = fopen("../../../input_own/matdata.txt", "r");
	//int vnum1, vnum2;
	int datam;
	//int flag, dim, row, col;
	//unsigned long long p1, p2;
	//unsigned char datause;

	fscanf(fpmr1, "%d %d %lld %lld %d %d", &flag, &dim, &p1, &p2, &row, &col);

	//image_test = image_left;


	unsigned char ch1[1241 * 376];

	//image_test.data = ch1;
	int imcol1 = col;
	for (int j = 0; j < row; j++)
	{
		for (int k = 0; k < col; k++)
		{
			fscanf(fpmr1, "%d %d", &vnum1, &vnum2);
			fscanf(fpmr1, "%d", &datam);

			datause = static_cast<unsigned char>(datam);
			int num = k + (j * imcol1);
			ch1[num] = datause;


		}
	}

	fclose(fpmr1);



	FILE* fpt1;
	fpt1 = fopen( "../../../input_own/matdata_check.txt", "w");



	fprintf(fpt1, "%d\t%d\t%d :\n",
		i,
		i,
		i);
	fprintf(fpt1, "flags: %d\tdims: %d\tstep: %lld\t",
		flag,
		dim,
		p1);
	//fprintf_s(fpt1, "%lld\trows: %d\tcols: %d \n ", image_test.step[1], image_test.rows, image_test.cols);
	fprintf(fpt1, "%lld\trows: %d\tcols: %d\size0: %d\size1: %d \n ", p2, row, col, size0, size1);


	//int imcol1 = image_test.cols;
	for (int j = 0; j < row; j++)
	{
		for (int k = 0; k < col; k++)
		{
			fprintf(fpt1, " ( ");
			fprintf(fpt1, "%d, ", j);
			fprintf(fpt1, "%d ): ", k);
			//fprintf(fpt1, " data: %d ", image_test.at<unsigned char>(j, k));
			fprintf(fpt1, " %d ", ch1[k + (j * imcol1)]);
			fprintf(fpt1, "\n");
		}
	}

	fclose(fpt1);


	//vector<Mat> trajectory_nolidar_bm =
	//	method.visual_odometry(mask);




	//for(int i = 0; i < 10; i++)
	    //cout<<"data: "<<static_cast<int>(ch1[i])<<endl;

	//vector<KeyPoint> kp0;
		float kp0[2 * 500];
		unsigned char des0[500 * 32];
		//cout << "image_test: " << image_test.step[1] << endl;

		extract_features(ch1, ch, kp0, des0);

		FILE* fp1;
			fp1 = fopen("../../../output_own/out_keypoint.txt", "w");


			fprintf(fp1, "%d\t%d\t%d :\n",
				i,
				i,
				i);
			for (int v = 0; v < 500; v++)
			{
				fprintf(fp1, "%d: ( ", v);
				fprintf(fp1, "%f, ", kp0[v * 2]);
				fprintf(fp1, "%f )", kp0[v * 2 + 1]);
				for (int descc = 0; descc < 32; descc++)
						{
							fprintf(fp1, "%d ",  des0[descc + (v * 32)] );
						}//fprintf(fp1, " class_id: %d ", kp0.at(v).class_id);
				fprintf(fp1, "\n");
			}

			fclose(fp1);




    return 0;
}

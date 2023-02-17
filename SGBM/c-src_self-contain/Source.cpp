#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "Methods.h"
#include "Mat.h"

using namespace std;

int main()
{
	int i = 1;
	cout << "i:" << i << endl;

	Mat image_left_test, image_right_test;
    Mat k_left_test, t_left_test, t_right_test;

#pragma warning(disable:4996)
    FILE* fpml;
    fpml = fopen("../input/matdata_left.txt", "r");
    int vnum1, vnum2;
    int data;
    int flag, dim, row, col;
    unsigned long long p1, p2;

    fscanf(fpml, "%d %d %lld %lld %d %d", &flag, &dim, &p1, &p2, &row, &col);


    image_left_test.flags = flag;
    image_left_test.dims = dim;
    image_left_test.step[0] = p1;
    image_left_test.step[1] = p2;
    image_left_test.rows = row;
    image_left_test.cols = col;
    static unsigned char ch1[1241 * 376];

    image_left_test.data = ch1;
    int imcol1 = image_left_test.cols;
    for (int j = 0; j < image_left_test.rows; j++)
    {
        for (int k = 0; k < image_left_test.cols; k++)
        {
            fscanf(fpml, "%d %d", &vnum1, &vnum2);
            fscanf(fpml, "%d", &data);

            int num = k + (j * imcol1);
            image_left_test.data[num] = static_cast<unsigned char>(data);


        }
    }

    fclose(fpml);

    //right------------------------------------------------------------------------------
    FILE* fpmr;
    fpmr = fopen("../input/matdata_right.txt", "r");
    int vnum1r, vnum2r;
    int datar;
    int flagr, dimr, rowr, colr;
    unsigned long long p1r, p2r;

    fscanf(fpmr, "%d %d %lld %lld %d %d", &flagr, &dimr, &p1r, &p2r, &rowr, &colr);


    image_right_test.flags = flagr;
    image_right_test.dims = dimr;
    image_right_test.step[0] = p1r;
    image_right_test.step[1] = p2r;
    image_right_test.rows = rowr;
    image_right_test.cols = colr;
    static unsigned char chr[1241 * 376];

    image_right_test.data = chr;
    int imcolr = image_right_test.cols;
    for (int j = 0; j < image_right_test.rows; j++)
    {
        for (int k = 0; k < image_right_test.cols; k++)
        {
            fscanf(fpmr, "%d %d", &vnum1r, &vnum2r);
            fscanf(fpmr, "%d", &datar);

            int numr = k + (j * imcolr);
            image_right_test.data[numr] = static_cast<unsigned char>(datar);


        }
    }

    fclose(fpmr);


    //k_left---------------------------------------------------------------------------

    FILE* fpmr1k;
    fpmr1k = fopen("../input/matdata_k_left.txt", "r");
    int vnum11k, vnum21k;
    float data1k;
    int flag1k, dim1k, row1k, col1k;
    unsigned long long p11k, p21k;
    //unsigned char datause;

    fscanf(fpmr1k, "%d %d %lld %lld %d %d", &flag1k, &dim1k, &p11k, &p21k, &row1k, &col1k);

    //image_test = image_left;

    k_left_test.flags = flag1k;
    k_left_test.dims = dim1k;
    k_left_test.step[0] = p11k;
    k_left_test.step[1] = p21k;
    k_left_test.rows = row1k;
    k_left_test.cols = col1k;
    static unsigned char ch1k[3 * 3 * 4];

    k_left_test.data = ch1k;
    //int imcol1k = k_left_test.cols;
    for (int j = 0; j < k_left_test.rows; j++)
    {
        for (int k = 0; k < k_left_test.cols; k++)
        {
            fscanf(fpmr1k, "%d %d", &vnum11k, &vnum21k);
            fscanf(fpmr1k, "%f", &data1k);


            //int num1k = k + (j * imcol1k);
            //k_left_test.data[num1k] = static_cast<unsigned char>(data1k);
            k_left_test.at<float>(j, k) = static_cast<float>(data1k);


        }
    }

    fclose(fpmr1k);


    //t_right----------------------------------------------------------------------
    FILE* fpmr1r;
    fpmr1r = fopen("../input/matdata_t_right.txt", "r");
    float data1r;
    int flag1r, dim1r, row1r, col1r;
    unsigned long long p11r, p21r;
    //unsigned char datause;

    fscanf(fpmr1r, "%d %d %lld %lld %d %d", &flag1r, &dim1r, &p11r, &p21r, &row1r, &col1r);

    //image_test = image_left;

    t_right_test.flags = flag1r;
    t_right_test.dims = dim1r;
    t_right_test.step[0] = p11r;
    t_right_test.step[1] = p21r;
    t_right_test.rows = row1r;
    t_right_test.cols = col1r;
    static unsigned char ch1r[4 * 1 * 4];

    t_right_test.data = ch1r;
    int imcol1k = t_right_test.cols;

    for (int k = 0; k < 16; k++)
    {
        fscanf(fpmr1r, "%f", &data1r);


        //int num1k = k + (j * imcol1k);
        t_right_test.data[k] = static_cast<unsigned char>(data1r);
        //t_right_test.at<float>(j, k) = static_cast<float>(data1r);

    }

    fclose(fpmr1r);



    //t_left-----------------------------------------------------------------------------
    FILE* fpmr1t;
    fpmr1t = fopen("../input/matdata_t_left.txt", "r");
    int vnum11t, vnum21t;
    float data1t;
    int flag1t, dim1t, row1t, col1t;
    unsigned long long p11t, p21t;
    //unsigned char datause;

    fscanf(fpmr1t, "%d %d %lld %lld %d %d", &flag1t, &dim1t, &p11t, &p21t, &row1t, &col1t);

    //image_test = image_left;

    t_left_test.flags = flag1t;
    t_left_test.dims = dim1t;
    t_left_test.step[0] = p11t;
    t_left_test.step[1] = p21t;
    t_left_test.rows = row1t;
    t_left_test.cols = col1t;
    static unsigned char ch1t[4 * 1 * 4];

    t_left_test.data = ch1t;
    //int imcol1k = k_left_test.cols;
    for (int j = 0; j < t_left_test.rows; j++)
    {
        for (int k = 0; k < t_left_test.cols; k++)
        {
            fscanf(fpmr1t, "%d %d", &vnum11t, &vnum21t);
            fscanf(fpmr1t, "%f", &data1t);


            //int num1k = k + (j * imcol1k);
            //k_left_test.data[num1k] = static_cast<unsigned char>(data1k);
            t_left_test.at<float>(j, k) = static_cast<float>(data1t);


        }
    }

    fclose(fpmr1t);

    //----------------------------------------------------------------------------------------

    Mat depth = stereo_2_depth(image_left_test, image_right_test, k_left_test, t_left_test, t_right_test, SGBM, false, true); 

    //----------------------------------------------------------------------------------------
    /*
    Mat disp_left = computeLeftDisparityMap(image_left_test, image_right_test, SGBM, 0);

    FILE* fpd;
    fopen_s(&fpd, "../output/test_disp.txt", "w");

    fprintf_s(fpd, "%d\t%d\t%lld\t%lld\t",
        disp_left.flags,
        disp_left.dims,
        disp_left.step[0],
        disp_left.step[1]
    );
    fprintf_s(fpd, "%d\t%d\n", disp_left.rows, disp_left.cols);

    for (int j = 0; j < disp_left.rows; j++)
    {
        for (int k = 0; k < disp_left.cols; k++)
        {
            fprintf_s(fpd, " %d ", j);
            fprintf_s(fpd, " %d ", k);
            fprintf_s(fpd, " %f ", disp_left.at<float>(j, k));
            fprintf_s(fpd, "\n");
        }
    }

    fclose(fpd);*/

    FILE* fpd2;
    fopen_s(&fpd2, "../output/test_depth.txt", "w");

    fprintf_s(fpd2, "%d\t%d\t%lld\t%lld\t",
        depth.flags,
        depth.dims,
        depth.step[0],
        depth.step[1]
    );
    fprintf_s(fpd2, "%d\t%d\n", depth.rows, depth.cols);

    for (int j = 0; j < depth.rows; j++)
    {
        for (int k = 0; k < depth.cols; k++)
        {
            fprintf_s(fpd2, " %d ", j);
            fprintf_s(fpd2, " %d ", k);
            fprintf_s(fpd2, " %f ", depth.at<float>(j, k));
            //cout << "type: " << depth.type() << endl;//5
            fprintf_s(fpd2, "\n");
        }
    }

    fclose(fpd2);


	
}


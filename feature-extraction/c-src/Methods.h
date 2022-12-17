#pragma once
#include<opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
class Dataset_Handler;
#include "Dataset_Handler.h"



using namespace std;
//using namespace cv;


#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0

#define CV_OCL_RUN_(condition, func, ...)
#define CV_OCL_RUN(condition, func) CV_OCL_RUN_(condition, func)
#ifndef __CV_EXPAND
#define __CV_EXPAND(x) x
#endif

/*#define CALL_HAL(name, fun, ...) {
										 \
	int res = __CV_EXPAND(fun(__VA_ARGS__)); \
	if (res == CV_HAL_ERROR_OK) \
		return; \
	else if (res != CV_HAL_ERROR_NOT_IMPLEMENTED) \
		CV_Error_(cv::Error::StsInternal, \
			("HAL implementation " CVAUX_STR(name) " ==> " CVAUX_STR(fun) " returned %d (0x%08x)", res, res)); \
}*/

class Methods
{
public:
	cv::Mat data;
	void read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1);
	vector<cv::Mat> groundTruthTrajectory(const char* file_name, int data_num);
	cv::Mat computeLeftDisparityMap(cv::Mat img_left, cv::Mat img_right, int matcher_name, bool rgb);
	void decompose_Projection_Matrix(cv::Mat p, cv::Mat* k, cv::Mat* r, cv::Mat* t);
	cv::Mat calc_depth_map(cv::Mat disp_left, cv::Mat k_left, cv::Mat t_left, cv::Mat t_right, bool rectified);
	cv::Mat stereo_2_depth(cv::Mat img_left, cv::Mat img_right, cv::Mat P0, cv::Mat P1, bool matcher, bool rgb, bool rectified);
	cv::Mat extract_features(cv::Mat image, int detector, cv::Mat mask, vector<cv::KeyPoint>* kp);
	vector<vector<cv::DMatch>> match_features(cv::Mat des1, cv::Mat des2, bool matching, int detector, bool sorting, int  k);
	vector<vector<cv::DMatch>> filter_matches_distance(vector<vector<cv::DMatch>> matches, float dist_threshold);
	void visualize_matches(cv::Mat image1, vector<cv::KeyPoint> kp1, cv::Mat image2, vector<cv::KeyPoint> kp2, vector<vector<cv::DMatch>> match);
	void estimate_motion(vector<vector<cv::DMatch>> match, vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, cv::Mat k, cv::Mat depth1, int max_depth,
		cv::Mat& rmat, cv::Mat& tvec, cv::Mat& image1_points, cv::Mat& image2_points);
	vector<cv::Mat> visual_odometry(Dataset_Handler handler, int detector, bool matching, float filter_match_distance, bool stereo_matcher, int subset, cv::Mat mask);

};
#pragma once
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>

using namespace std;

class FeatureMatching
{
private:
	void __knnMatch(cv::InputArray queryDescriptors, cv::InputArray trainDescriptors,
		vector<vector<cv::DMatch> >& matches, int k);

	void __batchDistance(cv::InputArray _src1, cv::InputArray _src2,
		cv::OutputArray _dist, int dtype, cv::OutputArray _nidx,
		int normType, int K);

	void __batchDistL2_(const float* src1, const float* src2, size_t step2,
		int nvecs, int len, float* dist);

	float __normL2Sqr(const float* a, const float* b, int n);

public:
	vector<cv::DMatch> match_features(cv::Mat des1, cv::Mat des2, int detector, bool sorting, float dist_threshold);

};


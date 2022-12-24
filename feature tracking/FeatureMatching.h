#pragma once
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>

using namespace std;

class FeatureMatching
{
private:
    void knnMatch(cv::InputArray queryDescriptors, cv::InputArray trainDescriptors,
        vector<vector<cv::DMatch> >& matches, int k);

    void batchDistance(cv::InputArray _src1, cv::InputArray _src2,
        cv::OutputArray _dist, int dtype, cv::OutputArray nidx,
        int normType, int K);

    void __batchDistHamming2(const uchar* src1, const uchar* src2, size_t step2,
        int nvecs, int len, int* dist);

    int __normHamming(const uchar* a, const uchar* b, int n, int cellSize);

public:
    vector<cv::DMatch> match_features(cv::Mat des1, cv::Mat des2, int detector, bool sorting, float dist_threshold);
};

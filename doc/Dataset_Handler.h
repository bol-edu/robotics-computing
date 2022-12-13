#pragma once
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tchar.h>
#include <vector>
#include "Methods.h"

using namespace std;

class Dataset_Handler
{
public:
    int seq;
    char *seq_dir;
    char *poses_dir;
    vector<cv::Mat> poses;
    vector<char *> left_image_files;
    vector<char *> right_image_files;
    int imheight;
    int imwidth;
    int num_frames;
    cv::Mat P0;
    cv::Mat P1;

    Dataset_Handler(int sequence);
    void read_data();
};
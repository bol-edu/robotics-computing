#pragma once
//#include <opencv2/opencv.hpp>
using namespace std;
//#include "SIFT__.h"



#define CV_32F_  5

#define DoG_TYPE_SHORT 0
#if DoG_TYPE_SHORT
// intermediate type used for DoG pyramids
typedef short sift_wt;
static const int SIFT_FIXPT_SCALE = 48;
#else
// intermediate type used for DoG pyramids
typedef float sift_wt;
static const int SIFT_FIXPT_SCALE = 1;
#endif

#define CV_CN_SHIFT_   3
#define CV_DEPTH_MAX_  (1 << CV_CN_SHIFT_)
#define CV_MAT_DEPTH_MASK_      (CV_DEPTH_MAX_ - 1)
#define CV_MAT_DEPTH_(flags)     ((flags) & CV_MAT_DEPTH_MASK_)
#define CV_MAKETYPE_(depth,cn) (CV_MAT_DEPTH_(depth) + (((cn)-1) << CV_CN_SHIFT_))

enum {

    INTER_LINEAR_ = 1

};

static const float SIFT_INIT_SIGMA = 0.5f;



template<typename _Tp> class DataType__
{
public:
#ifdef OPENCV_TRAITS_ENABLE_DEPRECATED
    typedef _Tp         value_type;
    typedef value_type  work_type;
    typedef value_type  channel_type;
    typedef value_type  vec_type;
    enum {
        generic_type = 1,
        depth = -1,
        channels = 1,
        fmt = 0,
        type = CV_MAKETYPE(depth, channels)
    };
#endif
};

template<> class DataType__<float>
{
public:
    typedef float       value_type;
    typedef value_type  work_type;
    typedef value_type  channel_type;
    typedef value_type  vec_type;
    enum {
        generic_type = 0,
        depth = CV_32F_,
        channels = 1,
        fmt = (int)'f',
        type = CV_MAKETYPE_(depth, channels)
    };
};



static Mat createInitialImage__(const Mat& img, bool doubleImageSize, float sigma)
{
    //CV_TRACE_FUNCTION();

    Mat gray, gray_fpt;
   /* if (img.channels() == 3 || img.channels() == 4)
    {
        cout << "cvtColor" << endl;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        gray.convertTo(gray_fpt, DataType__<sift_wt>::type, SIFT_FIXPT_SCALE, 0);
    }
    else*/
        img.convertTo(gray_fpt, DataType__<sift_wt>::type, SIFT_FIXPT_SCALE, 0);

    float sig_diff;

    if (doubleImageSize)
    {
        //cout << "doubleImageSize" << endl;
        sig_diff = sqrtf(max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4, 0.01f));
        Mat dbl;
#if DoG_TYPE_SHORT
        resize(gray_fpt, dbl, Size(gray_fpt.cols * 2, gray_fpt.rows * 2), 0, 0, INTER_LINEAR_EXACT);
#else
        resize__(gray_fpt, dbl, Size(gray_fpt.cols * 2, gray_fpt.rows * 2), 0, 0, INTER_LINEAR_);
#endif
        Mat result;
        GaussianBlur__(dbl, result, Size(), sig_diff, sig_diff);
        return result;
    }
    else
    {
        //cout << "else" << endl;
        sig_diff = sqrtf(max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA, 0.01f));
        Mat result;
        GaussianBlur__(gray_fpt, result, Size(), sig_diff, sig_diff);
        return result;
    }
}
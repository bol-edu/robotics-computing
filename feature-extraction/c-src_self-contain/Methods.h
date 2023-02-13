#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "define.h"
#include "Mat.h"
#include "KeyPoint.h"
#include "resize1.h"
#include "threshold__.h"
#include "copyMakeBorder__.h"
#include "GaussianBlur__.h"
#include "KeyPointsFilter__.h"
//#include "AutoBuffer__.h"
#include "RNG__.h"
//#include "BufferArea__.h"
#include "FAST__.h"
#include "Point__.h"








using namespace std;
//using namespace cv;


#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0


class Methods
{
public:

	//~Methods();
	//Mat data;
	/*void read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1);
	vector<cv::Mat> groundTruthTrajectory(const char* file_name, int data_num);
	cv::Mat computeLeftDisparityMap(cv::Mat img_left, cv::Mat img_right, int matcher_name, bool rgb);
	void decompose_Projection_Matrix(cv::Mat p, cv::Mat* k, cv::Mat* r, cv::Mat* t);
	cv::Mat calc_depth_map(cv::Mat disp_left, cv::Mat k_left, cv::Mat t_left, cv::Mat t_right, bool rectified);
	cv::Mat stereo_2_depth(cv::Mat img_left, cv::Mat img_right, cv::Mat P0, cv::Mat P1, bool matcher, bool rgb, bool rectified);*/
	//Mat extract_features(Mat image, Mat mask, vector<KeyPoint>* kp);
	/*vector<vector<cv::DMatch>> match_features(cv::Mat des1, cv::Mat des2, bool matching, int detector, bool sorting, int  k);
	vector<vector<cv::DMatch>> filter_matches_distance(vector<vector<cv::DMatch>> matches, float dist_threshold);
	void visualize_matches(cv::Mat image1, vector<cv::KeyPoint> kp1, cv::Mat image2, vector<cv::KeyPoint> kp2, vector<vector<cv::DMatch>> match);
	void estimate_motion(vector<vector<cv::DMatch>> match, vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, cv::Mat k, cv::Mat depth1, int max_depth,
		cv::Mat& rmat, cv::Mat& tvec, cv::Mat& image1_points, cv::Mat& image2_points);*/
	vector<Mat> visual_odometry(Mat mask);

};

vector<Mat> Methods::visual_odometry(Mat mask)
{


    vector<Mat> trajectory(1);

    Mat image_test;

    int i = 1;
    vector<KeyPoint> kp0, kp1, kpn0;

    //Mat des0 = extract_features(image_test, mask, &kp0);
    //cout << "des0: " << des0.dims << endl;
    //cout << "trajectory " << endl;

    return trajectory;
}






static void
computeOrbDescriptors(const Mat& imagePyramid, const std::vector<Rect>& layerInfo,
    const std::vector<float>& layerScale, std::vector<KeyPoint>& keypoints,
    Mat& descriptors, const std::vector<Point__>& _pattern, int dsize, int wta_k)
{
    int step = (int)imagePyramid.step[0];
    int j, i, nkeypoints = (int)keypoints.size();

    for (j = 0; j < nkeypoints; j++)
    {
        const KeyPoint& kpt = keypoints[j];
        const Rect& layer = layerInfo[kpt.octave];
        float scale = 1.f / layerScale[kpt.octave];
        float angle = kpt.angle; //angle = cvFloor(angle/12)*12.f;

        angle *= (float)(CV_PI / 180.f);
        float a = (float)cos(angle), b = (float)sin(angle);

        const uchar* center = &imagePyramid.at<uchar>(cvRound__(kpt.pt.y * scale) + layer.y,
            cvRound__(kpt.pt.x * scale) + layer.x);
        float x, y;
        int ix, iy;
        const Point__* pattern = &_pattern[0];
        uchar* desc = descriptors.ptr<uchar>(j);

#if 1                         //取旋轉後一個像素點的值
#define GET_VALUE(idx) (x = pattern[idx].x * a - pattern[idx].y * b, \
            y = pattern[idx].x * b + pattern[idx].y * a, \
            ix = cvRound__(x), \
            iy = cvRound__(y), \
            * (center + iy * step + ix))
#else
        //取旋轉後一個像素點，插值法
#define GET_VALUE(idx) (x = pattern[idx].x * a - pattern[idx].y * b, \
            y = pattern[idx].x * b + pattern[idx].y * a, \
            ix = cvFloor__(x), iy = cvFloor__(y), \
            x -= ix, y -= iy, \
            cvRound__(center[iy * step + ix] * (1 - x) * (1 - y) + center[(iy + 1) * step + ix] * (1 - x) * y + \
                center[iy * step + ix + 1] * x * (1 - y) + center[(iy + 1) * step + ix + 1] * x * y))
#endif

        if (wta_k == 2)
        {
            for (i = 0; i < dsize; ++i, pattern += 16) //每個特徵描述子長度爲32個字節
            {
                int t0, t1, val;
                t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                val = t0 < t1;
                t0 = GET_VALUE(2); t1 = GET_VALUE(3);
                val |= (t0 < t1) << 1;
                t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                val |= (t0 < t1) << 2;
                t0 = GET_VALUE(6); t1 = GET_VALUE(7);
                val |= (t0 < t1) << 3;
                t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                val |= (t0 < t1) << 4;
                t0 = GET_VALUE(10); t1 = GET_VALUE(11);
                val |= (t0 < t1) << 5;
                t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                val |= (t0 < t1) << 6;
                t0 = GET_VALUE(14); t1 = GET_VALUE(15);
                val |= (t0 < t1) << 7;

                desc[i] = (uchar)val;
            }
        }

        else
            ;// CV_Error(Error::StsBadSize, "Wrong wta_k. It can be only 2, 3 or 4.");
#undef GET_VALUE
    }
}



static void makeRandomPattern(int patchSize, Point__* pattern, int npoints)
{
    RNG__ rng(0x34985739); // we always start with a fixed seed,
    // to make patterns the same on each run
    for (int i = 0; i < npoints; i++)
    {
        pattern[i].x = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
        pattern[i].y = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
    }
}

static void ICAngles(const Mat& img, const std::vector<Rect>& layerinfo,
    std::vector<KeyPoint>& pts, const std::vector<int>& u_max, int half_k)
{
    int step = (int)img.step1();
    size_t ptidx, ptsize = pts.size();

    for (ptidx = 0; ptidx < ptsize; ptidx++)
    {
        const Rect& layer = layerinfo[pts[ptidx].octave];
        const uchar* center = &img.at<uchar>(cvRound__(pts[ptidx].pt.y) + layer.y, cvRound__(pts[ptidx].pt.x) + layer.x);

        int m_01 = 0, m_10 = 0;

        // Treat the center line differently, v=0
        for (int u = -half_k; u <= half_k; ++u)
            m_10 += u * center[u];

        // Go line by line in the circular patch
        for (int v = 1; v <= half_k; ++v)  //每次處理對稱的兩行v
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);  //計算m_01時,位置上差一個符號
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;  //計算上下兩行的m_01
        }

        pts[ptidx].angle = fastAtan2__((float)m_01, (float)m_10);  //計算角度
    }
}


const float HARRIS_K = 0.04f;

static void
HarrisResponses(const Mat& img, const std::vector<Rect>& layerinfo,
    std::vector<KeyPoint>& pts, int blockSize, float harris_k)
{
    //CV_CheckTypeEQ(img.type(), CV_8UC1, "");
    //CV_CheckGT(blockSize, 0, "");
    //CV_CheckLE(blockSize * blockSize, 2048, "");

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    size_t size_t_step = img.step[0];
    //CV_CheckLE(size_t_step * blockSize + blockSize + 1, (size_t)INT_MAX, "");  // ofs computation, step+1
    int step = static_cast<int>(size_t_step);

    int r = blockSize / 2;

    float scale = 1.f / ((1 << 2) * blockSize * 255.f);
    float scale_sq_sq = scale * scale * scale * scale;

    AutoBuffer__<int> ofsbuf(blockSize * blockSize);
    int* ofs = ofsbuf.data();
    for (int i = 0; i < blockSize; i++)
        for (int j = 0; j < blockSize; j++)
            ofs[i * blockSize + j] = (int)(i * step + j);

    for (ptidx = 0; ptidx < ptsize; ptidx++)
    {
        int x0 = cvRound__(pts[ptidx].pt.x);
        int y0 = cvRound__(pts[ptidx].pt.y);
        int z = pts[ptidx].octave;

        const uchar* ptr0 = ptr00 + (y0 - r + layerinfo[z].y) * size_t_step + (x0 - r + layerinfo[z].x);
        int a = 0, b = 0, c = 0;

        for (int k = 0; k < blockSize * blockSize; k++)
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[step - 1]);
            int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[-step + 1]);
            a += Ix * Ix;
            b += Iy * Iy;
            c += Ix * Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
            harris_k * ((float)a + b) * ((float)a + b)) * scale_sq_sq;
    }
}


void detect_fast(Mat& _image, std::vector<KeyPoint>& keypoints, Mat mask)
{
    /*enum DetectorType
    {
        TYPE_5_8 = 0, TYPE_7_12 = 1, TYPE_9_16 = 2
    };*/


    int threshold = 20;
    bool nonmaxSuppression = true;
    //DetectorType type = TYPE_9_16;

    //cout << "detect" << endl;

    if (_image.empty())
    {
        cout << "in detect_fast: _image is empty " << endl;
        keypoints.clear();
        return;
    }

    //Mat mask = _mask.getMat(), grayImage;

    Mat gray = _image;

    //outmat(gray);
    FAST__(gray, keypoints, threshold, nonmaxSuppression);
    KeyPointsFilter__::runByPixelsMask(keypoints, mask);
}


static void computeKeyPoints(const Mat& imagePyramid,

    const Mat& maskPyramid,
    const std::vector<Rect>& layerInfo,

    const std::vector<float>& layerScale,
    std::vector<KeyPoint>& allKeypoints,
    int nfeatures, double scaleFactor,
    int edgeThreshold, int patchSize, int scoreType,
    int fastThreshold)
{


    int i, nkeypoints, level, nlevels = (int)layerInfo.size(); //金字塔層數
    std::vector<int> nfeaturesPerLevel(nlevels);

    // fill the extractors and descriptors for the corresponding scales
    float factor = (float)(1.0 / scaleFactor);
    float ndesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)std::pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for (level = 0; level < nlevels - 1; level++) //對每層圖像上分配相應角點數
    {
        nfeaturesPerLevel[level] = cvRound__(ndesiredFeaturesPerScale);
        sumFeatures += nfeaturesPerLevel[level];
        ndesiredFeaturesPerScale *= factor;
    }
    nfeaturesPerLevel[nlevels - 1] = max(nfeatures - sumFeatures, 0); //剩下角點數，由最上層圖像提取

    // Make sure we forget about what is too close to the boundary
    //edge_threshold_ = std::max(edge_threshold_, patch_size_/2 + kKernelWidth / 2 + 2);

    // pre-compute the end of a row in a circular patch
    int halfPatchSize = patchSize / 2;  //計算每個特徵點圓鄰域的位置信息
    std::vector<int> umax(halfPatchSize + 2);

    int v, v0, vmax = cvFloor__(halfPatchSize * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil__(halfPatchSize * std::sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }

    allKeypoints.clear();
    std::vector<KeyPoint> keypoints;
    std::vector<int> counters(nlevels);
    keypoints.reserve(nfeaturesPerLevel[0] * 2);
    //cout << "C: " << nfeaturesPerLevel[0] * 2 << endl;

    for (level = 0; level < nlevels; level++)
    {
        int featuresNum = nfeaturesPerLevel[level];
        Mat img = imagePyramid(layerInfo[level]);
        Mat mask = maskPyramid.empty() ? Mat() : maskPyramid(layerInfo[level]);


        // Detect FAST features, 20 is a good threshold

            //Ptr<FastFeatureDetector__> fd = FastFeatureDetector__::create(fastThreshold, true);
            //fd->detect(img, keypoints, mask); //Fast角點檢測



            detect_fast(img, keypoints, mask); //Fast角點檢測





        // Remove keypoints very close to the border

        Size img_size(img.size[1], img.size[0]);
        KeyPointsFilter__::runByImageBorder(keypoints, img_size, edgeThreshold); //去除鄰近邊界的點

        // Keep more points than necessary as FAST does not give amazing corners
        KeyPointsFilter__::retainBest(keypoints, scoreType == 0 ? 2 * featuresNum : featuresNum);
        //按Fast強度排序,保留前2*featuresNum個特徵點

        nkeypoints = (int)keypoints.size();
        counters[level] = nkeypoints;

        float sf = layerScale[level];
        for (i = 0; i < nkeypoints; i++) // Set the level of the coordinates
        {
            keypoints[i].octave = level;  //層信息
            keypoints[i].size = patchSize * sf;
        }

        std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(allKeypoints));

    }



    nkeypoints = (int)allKeypoints.size();
    if (nkeypoints == 0)
    {
        return;
    }
    Mat responses;


    // Select best features using the Harris cornerness (better scoring than FAST)
    //if (scoreType == ORB_Impl__::HARRIS_SCORE)
    //{

        HarrisResponses(imagePyramid, layerInfo, allKeypoints, 7, HARRIS_K);
        //計算每個角點的Harris強度響應

        std::vector<KeyPoint> newAllKeypoints;
        newAllKeypoints.reserve(nfeaturesPerLevel[0] * nlevels);

        int offset = 0;
        for (level = 0; level < nlevels; level++)
        {
            int featuresNum = nfeaturesPerLevel[level];
            nkeypoints = counters[level];
            keypoints.resize(nkeypoints);
            std::copy(allKeypoints.begin() + offset,
                allKeypoints.begin() + offset + nkeypoints,
                keypoints.begin());
            offset += nkeypoints;

            //cull to the final desired level, using the new Harris scores.
            KeyPointsFilter__::retainBest(keypoints, featuresNum); //按Harris強度排序，保留前featuresNum個

            std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(newAllKeypoints));
        }
        std::swap(allKeypoints, newAllKeypoints);
    //}

    nkeypoints = (int)allKeypoints.size();


    {   // Process each keypoint  爲每個角點計算主方向，質心法
        ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize);
    }

    for (i = 0; i < nkeypoints; i++)
    {
        float scale = layerScale[allKeypoints[i].octave];
        allKeypoints[i].pt *= scale;
    }
}




static inline size_t alignSize__(size_t sz, int n)
{
    return (sz + n - 1) & -n;
}

static inline float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}

void detectAndCompute(Mat image, Mat mask,
	std::vector<KeyPoint>& keypoints,
	/*OutputArray*/ Mat& descriptors, bool useProvidedKeypoints)
{
	enum ScoreType { HARRIS_SCORE = 0 };
	int nfeatures = 500;
	float scaleFactor = 1.2f;
	int nlevels = 8;
	int edgeThreshold = 31;
	int firstLevel = 0;
	int wta_k = 2;
	int scoreType = HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;


	bool do_keypoints = !useProvidedKeypoints;
	bool do_descriptors = descriptors.dims == 0 ? false : true;

	cout << "do_keypoints: " << do_keypoints << endl;
	cout << "do_descriptors: " << do_descriptors << endl;

    if ((!do_keypoints && !do_descriptors) || image.empty())
        return;


    const int HARRIS_BLOCK_SIZE = 9; //Harris角點響應需要的邊界大小
    int halfPatchSize = patchSize / 2; //鄰域半徑
    // sqrt(2.0) is for handling patch rotation
    int descPatchSize = cvCeil__(halfPatchSize * sqrt(2.0));
    int border = std::max(edgeThreshold, std::max(descPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1; //採用最大的邊界

    //cout << "border: " << border << endl;// 32


    bool useOCL = false;
    //#endif

   // Mat image = _image.getMat(), mask = _mask.getMat();


    int i, nLevels = nlevels, level, nkeypoints = (int)keypoints.size(); //金字塔層數
    bool sortedByLevel = true;

    if (!do_keypoints) //不做特徵點檢測
    {

        nLevels = 0;
        for (i = 0; i < nkeypoints; i++)
        {
            level = keypoints[i].octave;

            if (i > 0 && level < keypoints[i - 1].octave)
                sortedByLevel = false;
            nLevels = std::max(nLevels, level);
        }
        nLevels++;
    }

    std::vector<Rect> layerInfo(nLevels);
    std::vector<int> layerOfs(nLevels);
    std::vector<float> layerScale(nLevels);
    Mat imagePyramid, maskPyramid;  //創建尺度金字塔圖像

    float level0_inv_scale = 1.0f / getScale(0, firstLevel, scaleFactor);       //cout << "level0_inv_scale: " << level0_inv_scale << endl; 1
    size_t level0_width = (size_t)cvRound__(image.cols * level0_inv_scale);     //cout << "level0_width: " << level0_width << endl; 1241
    size_t level0_height = (size_t)cvRound__(image.rows * level0_inv_scale);    //cout << "level0_height: " << level0_height << endl; 376
    Size bufSize((int)alignSize__(level0_width + border * 2, 16), 0);  // TODO change alignment to 64
    //cout << "bufSize: " << bufSize.height << " " << bufSize.width << endl; 0 1312

    int level_dy = (int)level0_height + border * 2;     //cout << "level_dy: " << level_dy << endl; 440
    Point__ level_ofs(0, 0);

    for (level = 0; level < nLevels; level++)
    {
        float scale = getScale(level, firstLevel, scaleFactor); //每層對應的尺度
        layerScale[level] = scale;
        float inv_scale = 1.0f / scale;
        Size sz(cvRound__(image.cols * inv_scale), cvRound__(image.rows * inv_scale));//每層對應的圖像大小
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        if (level_ofs.x + wholeSize.width > bufSize.width)
        {
            level_ofs = Point__(0, level_ofs.y + level_dy);
            level_dy = wholeSize.height;
        }
        //cout << "L: " << level << endl;
        Rect linfo(level_ofs.x + border, level_ofs.y + border, sz.width, sz.height);    // cout << "Rect linfo: " << linfo.x << " " << linfo.y << " " << linfo.width << " " << linfo.height << endl;
        layerInfo[level] = linfo;
        layerOfs[level] = linfo.y * bufSize.width + linfo.x;
        level_ofs.x += wholeSize.width;

    }
   bufSize.height = level_ofs.y + level_dy;
   cout << "bufSize: " << bufSize.height << " " << bufSize.width << endl;

    imagePyramid.create();


    if (!mask.empty())
        maskPyramid.create1();






    Mat prevImg = image, prevMask = mask;
    //outmat(prevImg);


    // Pre-compute the scale pyramids
    for (level = 0; level < nLevels; ++level)
    {



        Rect linfo = layerInfo[level];
        Size sz(linfo.width, linfo.height);
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        Rect wholeLinfo = Rect(linfo.x - border, linfo.y - border, wholeSize.width, wholeSize.height);
        Mat extImg = imagePyramid(wholeLinfo), extMask;
        Mat currImg = extImg(Rect(border, border, sz.width, sz.height)), currMask;

        //cout << "L: " << level << endl;
        //cout << "extImg: " << extImg.flags << endl;//112404704
        //cout << "currImg: " << currImg.cols << " " << currImg.rows << " " << currImg.size[0] << endl;
        //Mat prevImg1 = currImg, prevMask1 = currMask;






        if (!mask.empty())
        {
            extMask = maskPyramid(wholeLinfo);
            currMask = extMask(Rect(border, border, sz.width, sz.height));
        }

        // Compute the resized image
        if (level != firstLevel)  //得到金字塔每層的圖像
        {
           // if (level == 1)
           // {
                resize__(prevImg, currImg, sz, 0, 0, INTER_LINEAR_EXACT);
           // }
          //  else
          //  {
         //       resize__(prevImg1, currImg, sz, 0, 0, INTER_LINEAR_EXACT);
         //   }



            //cout << "currImgR : " << currImg.rows << " currImgC : " << currImg.cols << "currImgD : " << currImg.dims << endl;
            //cout << "size : " << *prevImg.size.p << endl;
            //cout << "data : " << *prevImg.data << endl;


            if (!mask.empty())
            {
                //cout << "!mask.empty()" << endl;
                //if (level == 1)
                //{
                    resize__(prevMask, currMask, sz, 0, 0, INTER_LINEAR_EXACT);
                //}
                //else
                //{
                //    resize__(prevMask1, currMask, sz, 0, 0, INTER_LINEAR_EXACT);
                //}
                //resize__(prevMask, currMask, sz, 0, 0, INTER_LINEAR_EXACT);
                    if (level > firstLevel)
                       threshold__(currMask, currMask, 254, 0, THRESH_TOZERO);
            }

            copyMakeBorder__(currImg, extImg, border, border, border, border,
                BORDER_REFLECT_101 + BORDER_ISOLATED);
            if (!mask.empty())
                copyMakeBorder__(currMask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);




        }
        else
        {

            copyMakeBorder__(image, extImg, border, border, border, border, BORDER_REFLECT_101); //擴大圖像的邊界
            //outmat(extImg);
           // outmat(imagePyramid);


            if (!mask.empty())
                copyMakeBorder__(mask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);


        }
        if (level > firstLevel)
        {
            prevImg = currImg;
            prevMask = currMask;

        }


        //Mat prevImg_save = currImg, prevMask_save = currMask;



    }


    //cout << "use: " << useOCL << endl;
    //cout << "layerInfo: " << layerInfo[level].x << " " << layerInfo[level].y << " " << layerInfo[level].height << " " << layerInfo[level].width << endl;



    if (do_keypoints) //提取角點
    {

        //outmat(imagePyramid);

            // Get keypoints, those will be far enough from the border that no check will be required for the descriptor
        computeKeyPoints(imagePyramid, maskPyramid,
            layerInfo, layerScale, keypoints,
            nfeatures, scaleFactor, edgeThreshold, patchSize, scoreType, fastThreshold);
        //cout << "imagePyramid : " << imagePyramid.rows << " imagePyramidC : " << imagePyramid.cols << "imagePyramidD : " << imagePyramid.dims << endl;
        //cout << "_descriptors : " << *_descriptors.data << endl;
    }
    else
    {
        Size image_size(image.size[1], image.size[0]);
        KeyPointsFilter__::runByImageBorder(keypoints, image_size, edgeThreshold);

        if (!sortedByLevel)
        {
            std::vector<std::vector<KeyPoint> > allKeypoints(nLevels);
            nkeypoints = (int)keypoints.size();
            for (i = 0; i < nkeypoints; i++)
            {
                level = keypoints[i].octave;
                //CV_Assert(0 <= level);
                allKeypoints[level].push_back(keypoints[i]);
            }
            keypoints.clear();
            for (level = 0; level < nLevels; level++)
                std::copy(allKeypoints[level].begin(), allKeypoints[level].end(), std::back_inserter(keypoints));
        }
    }

    if (do_descriptors)
    {
        //cout << "doing" << endl;
        int dsize = 32; //int dsize = descriptorSize();

        nkeypoints = (int)keypoints.size();
        if (nkeypoints == 0)
        {
            descriptors.release();
            return;
        }

        descriptors.create_des();
        std::vector<Point__> pattern;

        const int npoints = 512;
        Point__ patternbuf[npoints];
        const Point__* pattern0 = (const Point__*)bit_pattern_31_;

        if (patchSize != 31)
        {
            pattern0 = patternbuf;
            makeRandomPattern(patchSize, patternbuf, npoints);
        }


         //cout << "wta_k: " << wta_k << endl; 2
        if (wta_k == 2)
            std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


        for (level = 0; level < nLevels; level++)
        {
            // preprocess the resized image
            Mat workingMat = imagePyramid(layerInfo[level]);

            //boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
            GaussianBlur__(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);


        }



            //Mat descriptors = _descriptors.getMat();
            computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
                keypoints, descriptors, pattern, dsize, wta_k);


    }


}



void detect(Mat image,
	std::vector<KeyPoint>& keypoints,
	Mat mask)
{

	/*if (image.empty())
	{
		keypoints.clear();
		return;
	}*/
	Mat noArray;
	detectAndCompute(image, mask, keypoints, noArray, false);
}

void compute(Mat image,
	std::vector<KeyPoint>& keypoints,
	 Mat& descriptors)
{


	if (image.empty())
	{
		descriptors.release();
		return;
	}
	Mat noArray;
	cout << "descriptors: " << descriptors.dims << endl;
	descriptors.dims = 2;
	cout << "descriptors: " << descriptors.dims << endl;

	detectAndCompute(image, noArray, keypoints, descriptors, true);
}


Mat extract_features(Mat image, Mat mask, vector<KeyPoint>* kp)
{


	Mat des;
	des.dims = 1;

	detect(image, *kp, mask);
    cout << "detect end" << endl;
	compute(image, *kp, des);
    cout << "compute end" << endl;
	//cout << "extract_features" << endl;


	return des;



}





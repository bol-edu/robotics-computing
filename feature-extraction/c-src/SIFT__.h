#pragma once
#include <opencv2/opencv.hpp>
//#include "createInitialImage__.h"

using namespace std;

static const int SIFT_DESCR_WIDTH = 4;
static const int SIFT_DESCR_HIST_BINS = 8;
static const float SIFT_INIT_SIGMA = 0.5f;
static const int SIFT_IMG_BORDER = 5;

enum InterpolationFlags {

    INTER_NEAREST_ = 0,

    INTER_LINEAR_ = 1,

    /*INTER_CUBIC = 2,

    INTER_AREA = 3,

    INTER_LANCZOS4 = 4,

    INTER_LINEAR_EXACT = 5,

    INTER_NEAREST_EXACT = 6,

    INTER_MAX = 7,

    WARP_FILL_OUTLIERS = 8,

    WARP_INVERSE_MAP = 16*/
};


class CV_EXPORTS_W SIFT__ : public Feature2D
{
public:
    
    CV_WRAP static Ptr<SIFT__> create(int nfeatures = 0, int nOctaveLayers = 3,
        double contrastThreshold = 0.04, double edgeThreshold = 10,
        double sigma = 1.6);

    
    /*CV_WRAP static Ptr<SIFT__> create(int nfeatures, int nOctaveLayers,
        double contrastThreshold, double edgeThreshold,
        double sigma, int descriptorType);*/

    //CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;
};

//typedef SIFT__ SiftFeatureDetector;
//typedef SIFT__ SiftDescriptorExtractor;

class SIFT_Impl__ : public SIFT__
{
public:
    explicit SIFT_Impl__(int nfeatures = 0, int nOctaveLayers = 3,
        double contrastThreshold = 0.04, double edgeThreshold = 10,
        double sigma = 1.6, int descriptorType = CV_32F);

    //! returns the descriptor size in floats (128)
    int descriptorSize() const CV_OVERRIDE;

    //! returns the descriptor type
    //int descriptorType() const CV_OVERRIDE;

    //! returns the default norm type
    //int defaultNorm() const CV_OVERRIDE;

    //! finds the keypoints and computes descriptors for them using SIFT algorithm.
    //! Optionally it can compute descriptors for the user-provided keypoints
    void detectAndCompute(InputArray img, InputArray mask,
        std::vector<KeyPoint>& keypoints,
        OutputArray descriptors,
        bool useProvidedKeypoints = false) CV_OVERRIDE;

    void buildGaussianPyramid(const Mat& base, std::vector<Mat>& pyr, int nOctaves) const;
    void buildDoGPyramid(const std::vector<Mat>& pyr, std::vector<Mat>& dogpyr) const;
    void findScaleSpaceExtrema(const std::vector<Mat>& gauss_pyr, const std::vector<Mat>& dog_pyr,
        std::vector<KeyPoint>& keypoints) const;

protected:
    CV_PROP_RW int nfeatures;
    CV_PROP_RW int nOctaveLayers;
    CV_PROP_RW double contrastThreshold;
    CV_PROP_RW double edgeThreshold;
    CV_PROP_RW double sigma;
    CV_PROP_RW int descriptor_type;
};

Ptr<SIFT__> SIFT__::create(int _nfeatures, int _nOctaveLayers,
    double _contrastThreshold, double _edgeThreshold, double _sigma)
{
    //CV_TRACE_FUNCTION();

    return makePtr<SIFT_Impl__>(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma, CV_32F);
}

/*Ptr<SIFT__> SIFT__::create(int _nfeatures, int _nOctaveLayers,
    double _contrastThreshold, double _edgeThreshold, double _sigma, int _descriptorType)
{
    //CV_TRACE_FUNCTION();

    // SIFT descriptor supports 32bit floating point and 8bit unsigned int.
    CV_Assert(_descriptorType == CV_32F || _descriptorType == CV_8U);
    return makePtr<SIFT_Impl__>(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma, _descriptorType);
}*/

SIFT_Impl__::SIFT_Impl__(int _nfeatures, int _nOctaveLayers,
    double _contrastThreshold, double _edgeThreshold, double _sigma, int _descriptorType)
    : nfeatures(_nfeatures), nOctaveLayers(_nOctaveLayers),
    contrastThreshold(_contrastThreshold), edgeThreshold(_edgeThreshold), sigma(_sigma), descriptor_type(_descriptorType)
{
    cout << "SIFT_Impl__" << endl;
}

int SIFT_Impl__::descriptorSize() const
{
    return SIFT_DESCR_WIDTH * SIFT_DESCR_WIDTH * SIFT_DESCR_HIST_BINS;
}

static inline void
unpackOctave(const KeyPoint& kpt, int& octave, int& layer, float& scale)
{
    octave = kpt.octave & 255;
    layer = (kpt.octave >> 8) & 255;
    octave = octave < 128 ? octave : (-128 | octave);
    scale = octave >= 0 ? 1.f / (1 << octave) : (float)(1 << -octave);
}

static void calcDescriptors(const std::vector<Mat>& gpyr, const std::vector<KeyPoint>& keypoints,
    Mat& descriptors, int nOctaveLayers, int firstOctave)
{
    //CV_TRACE_FUNCTION();
    parallel_for_(Range(0, static_cast<int>(keypoints.size())), calcDescriptorsComputer(gpyr, keypoints, descriptors, nOctaveLayers, firstOctave));
}

void SIFT_Impl__::detectAndCompute(InputArray _image, InputArray _mask,
    std::vector<KeyPoint>& keypoints,
    OutputArray _descriptors,
    bool useProvidedKeypoints)
{
    //CV_TRACE_FUNCTION();

    int firstOctave = -1, actualNOctaves = 0, actualNLayers = 0;
    Mat image = _image.getMat(), mask = _mask.getMat();

    if (image.empty() || image.depth() != CV_8U)
        CV_Error(Error::StsBadArg, "image is empty or has incorrect depth (!=CV_8U)");

    if (!mask.empty() && mask.type() != CV_8UC1)
        CV_Error(Error::StsBadArg, "mask has incorrect type (!=CV_8UC1)");

    if (useProvidedKeypoints)
    {
        firstOctave = 0;
        int maxOctave = INT_MIN;
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            int octave, layer;
            float scale;
            unpackOctave(keypoints[i], octave, layer, scale);
            firstOctave = min(firstOctave, octave);
            maxOctave = max(maxOctave, octave);
            actualNLayers = max(actualNLayers, layer - 2);
        }

        firstOctave = min(firstOctave, 0);
        CV_Assert(firstOctave >= -1 && actualNLayers <= nOctaveLayers);
        actualNOctaves = maxOctave - firstOctave + 1;
    }

    Mat base = createInitialImage__(image, firstOctave < 0, (float)sigma);
    std::vector<Mat> gpyr;
    int nOctaves = actualNOctaves > 0 ? actualNOctaves : cvRound(std::log((double)min(base.cols, base.rows)) / std::log(2.) - 2) - firstOctave;

    //double t, tf = getTickFrequency();
    //t = (double)getTickCount();
    buildGaussianPyramid(base, gpyr, nOctaves);

    //t = (double)getTickCount() - t;
    //printf("pyramid construction time: %g\n", t*1000./tf);

    if (!useProvidedKeypoints)
    {
        std::vector<Mat> dogpyr;
        buildDoGPyramid(gpyr, dogpyr);
        //t = (double)getTickCount();
        findScaleSpaceExtrema(gpyr, dogpyr, keypoints);
        KeyPointsFilter::removeDuplicatedSorted(keypoints);

        if (nfeatures > 0)
            KeyPointsFilter::retainBest(keypoints, nfeatures);
        //t = (double)getTickCount() - t;
        //printf("keypoint detection time: %g\n", t*1000./tf);

        if (firstOctave < 0)
            for (size_t i = 0; i < keypoints.size(); i++)
            {
                KeyPoint& kpt = keypoints[i];
                float scale = 1.f / (float)(1 << -firstOctave);
                kpt.octave = (kpt.octave & ~255) | ((kpt.octave + firstOctave) & 255);
                kpt.pt *= scale;
                kpt.size *= scale;
            }

        if (!mask.empty())
            KeyPointsFilter::runByPixelsMask(keypoints, mask);
    }
    else
    {
        // filter keypoints by mask
        //KeyPointsFilter::runByPixelsMask( keypoints, mask );
    }

    if (_descriptors.needed())
    {
        //t = (double)getTickCount();
        int dsize = descriptorSize();
        _descriptors.create((int)keypoints.size(), dsize, descriptor_type);

        Mat descriptors = _descriptors.getMat();
        calcDescriptors(gpyr, keypoints, descriptors, nOctaveLayers, firstOctave);
        //t = (double)getTickCount() - t;
        //printf("descriptor extraction time: %g\n", t*1000./tf);
    }
}


void SIFT_Impl__::buildGaussianPyramid(const Mat& base, std::vector<Mat>& pyr, int nOctaves) const
{
    //CV_TRACE_FUNCTION();

    std::vector<double> sig(nOctaveLayers + 3);
    pyr.resize(nOctaves * (nOctaveLayers + 3));

    // precompute Gaussian sigmas using the following formula:
    //  \sigma_{total}^2 = \sigma_{i}^2 + \sigma_{i-1}^2
    sig[0] = sigma;
    double k = std::pow(2., 1. / nOctaveLayers);
    for (int i = 1; i < nOctaveLayers + 3; i++)
    {
        double sig_prev = std::pow(k, (double)(i - 1)) * sigma;
        double sig_total = sig_prev * k;
        sig[i] = std::sqrt(sig_total * sig_total - sig_prev * sig_prev);
    }

    for (int o = 0; o < nOctaves; o++)
    {
        for (int i = 0; i < nOctaveLayers + 3; i++)
        {
            Mat& dst = pyr[o * (nOctaveLayers + 3) + i];
            if (o == 0 && i == 0)
                dst = base;
            // base of new octave is halved image from end of previous octave
            else if (i == 0)
            {
                const Mat& src = pyr[(o - 1) * (nOctaveLayers + 3) + nOctaveLayers];
                resize(src, dst, Size(src.cols / 2, src.rows / 2),
                    0, 0, INTER_NEAREST_);
            }
            else
            {
                const Mat& src = pyr[o * (nOctaveLayers + 3) + i - 1];
                GaussianBlur(src, dst, Size(), sig[i], sig[i]);
            }
        }
    }
}


void SIFT_Impl__::buildDoGPyramid(const std::vector<Mat>& gpyr, std::vector<Mat>& dogpyr) const
{
    //CV_TRACE_FUNCTION();

    int nOctaves = (int)gpyr.size() / (nOctaveLayers + 3);
    dogpyr.resize(nOctaves * (nOctaveLayers + 2));

    parallel_for_(Range(0, nOctaves * (nOctaveLayers + 2)), buildDoGPyramidComputer(nOctaveLayers, gpyr, dogpyr));
}

void SIFT_Impl__::findScaleSpaceExtrema(const std::vector<Mat>& gauss_pyr, const std::vector<Mat>& dog_pyr,
    std::vector<KeyPoint>& keypoints) const
{
    //CV_TRACE_FUNCTION();

    const int nOctaves = (int)gauss_pyr.size() / (nOctaveLayers + 3);
    const int threshold = cvFloor(0.5 * contrastThreshold / nOctaveLayers * 255 * SIFT_FIXPT_SCALE);

    keypoints.clear();
    TLSDataAccumulator<std::vector<KeyPoint> > tls_kpts_struct;

    for (int o = 0; o < nOctaves; o++)
        for (int i = 1; i <= nOctaveLayers; i++)
        {
            const int idx = o * (nOctaveLayers + 2) + i;
            const Mat& img = dog_pyr[idx];
            const int step = (int)img.step1();
            const int rows = img.rows, cols = img.cols;

            parallel_for_(Range(SIFT_IMG_BORDER, rows - SIFT_IMG_BORDER),
                findScaleSpaceExtremaComputer(
                    o, i, threshold, idx, step, cols,
                    nOctaveLayers,
                    contrastThreshold,
                    edgeThreshold,
                    sigma,
                    gauss_pyr, dog_pyr, tls_kpts_struct));
        }

    std::vector<std::vector<KeyPoint>*> kpt_vecs;
    tls_kpts_struct.gather(kpt_vecs);
    for (size_t i = 0; i < kpt_vecs.size(); ++i) {
        keypoints.insert(keypoints.end(), kpt_vecs[i]->begin(), kpt_vecs[i]->end());
    }
}


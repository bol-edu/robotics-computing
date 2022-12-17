#include "Methods.h"
#include <fstream>
#include <stdlib.h>

#include "resize1.h"
#include "threshold__.h"
#include "copyMakeBorder__.h"
#include "GaussianBlur__.h"
#include "KeyPointsFilter__.h"
#include "AutoBuffer__.h"
//#include "RNG__.h"
#include "BufferArea__.h"
//#include "FAST__.h"
#include "Point__.h"
//#include "SIFT__.h"
#include "createInitialImage__.h"
#include "buildDoGPyramidComputer.h"
#include "findScaleSpaceExtremaComputer.h"
#include "calcDescriptorsComputer.h"

#include "TLSDataAccumulator.h"
#include "parallel_for__.h"
#include "Vec3f__.h"






using namespace std;
using namespace cv;
using namespace utils;
//#pragma warning( disable : 4996 )




class CV_EXPORTS Algorithm__;
//template<typename _Tp, typename _EnumTp = void> struct ParamType {};

class CV_EXPORTS_W Algorithm__
{
public:
    Algorithm__();
    virtual ~Algorithm__();

    /** @brief Clears the algorithm state
    */
    //CV_WRAP virtual void clear() {}

    /** @brief Stores algorithm parameters in a file storage
    */
    virtual void write(FileStorage& fs) const { CV_UNUSED(fs); }

    /** @brief simplified API for language bindings
    * @overload
    */
    CV_WRAP void write(const Ptr<FileStorage>& fs, const String& name = String()) const;

    /** @brief Reads algorithm parameters from a file storage
    */
    CV_WRAP virtual void read(const FileNode& fn) { CV_UNUSED(fn); }

    /** @brief Returns true if the Algorithm is empty (e.g. in the very beginning or after unsuccessful read
    */
    //CV_WRAP virtual bool empty() const { return false; }

    
    template<typename _Tp> static Ptr<_Tp> read(const FileNode& fn)
    {
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }

    
    template<typename _Tp> static Ptr<_Tp> load(const String& filename, const String& objname = String())
    {
        FileStorage fs(filename, FileStorage::READ);
        CV_Assert(fs.isOpened());
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        if (fn.empty()) return Ptr<_Tp>();
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }

    
    template<typename _Tp> static Ptr<_Tp> loadFromString(const String& strModel, const String& objname = String())
    {
        FileStorage fs(strModel, FileStorage::READ + FileStorage::MEMORY);
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }

    /** Saves the algorithm to a file.
    In order to make this method work, the derived class must implement Algorithm::write(FileStorage& fs). */
    CV_WRAP virtual void save(const String& filename) const;

    /** Returns the algorithm string identifier.
    This string is used as top level xml/yml node tag when the object is saved to a file or string. */
    CV_WRAP virtual String getDefaultName() const;

protected:
    void writeFormat(FileStorage& fs) const;
};

Algorithm__::Algorithm__()
{
    
    //CV_TRACE_FUNCTION();
}

Algorithm__::~Algorithm__()
{
    //CV_TRACE_FUNCTION();
}

void Algorithm__::write(const Ptr<FileStorage>& fs, const String& name) const
{
    //CV_TRACE_FUNCTION();
    if (name.empty())
    {
        write(*fs);
        return;
    }
    *fs << name << "{";
    write(*fs);
    *fs << "}";
}

void Algorithm__::save(const String& filename) const
{
    //CV_TRACE_FUNCTION();
    FileStorage fs(filename, FileStorage::WRITE);
    fs << getDefaultName() << "{";
    write(fs);
    fs << "}";
}

String Algorithm__::getDefaultName() const
{
    //CV_TRACE_FUNCTION();
    return String("my_object");
}

void Algorithm__::writeFormat(FileStorage& fs) const
{
    //CV_TRACE_FUNCTION();
    fs << "format" << (int)3;
}





/*class Feature2D__
{
public:
    virtual ~Feature2D__();

    virtual void detect(InputArray image,
        std::vector<KeyPoint>& keypoints,
        InputArray mask = noArray());

    virtual void compute(InputArray image,
        std::vector<KeyPoint>& keypoints,
        OutputArray descriptors);

    // Detects keypoints and computes the descriptors 
    virtual void detectAndCompute(InputArray image, InputArray mask,
        std::vector<KeyPoint>& keypoints,
        OutputArray descriptors,
        bool useProvidedKeypoints = false);
};*/

#ifdef __EMSCRIPTEN__
class CV_EXPORTS_W Feature2D__ : public Algorithm__
#else
class CV_EXPORTS_W Feature2D__ : public virtual Algorithm__
#endif
{
public:
    virtual ~Feature2D__();

    
    CV_WRAP virtual void detect(InputArray image,
         std::vector<KeyPoint>& keypoints,
        InputArray mask = noArray());

    
    /*CV_WRAP virtual void detect(InputArrayOfArrays images,
        CV_OUT std::vector<std::vector<KeyPoint> >& keypoints,
        InputArrayOfArrays masks = noArray());*/

   
    CV_WRAP virtual void compute(InputArray image,
          std::vector<KeyPoint>& keypoints,
        OutputArray descriptors);

    
    /*CV_WRAP virtual void compute(InputArrayOfArrays images,
        CV_OUT CV_IN_OUT std::vector<std::vector<KeyPoint> >& keypoints,
        OutputArrayOfArrays descriptors);*/

    /** Detects keypoints and computes the descriptors */
    CV_WRAP virtual void detectAndCompute(InputArray image, InputArray mask,
         std::vector<KeyPoint>& keypoints,
        OutputArray descriptors,
        bool useProvidedKeypoints = false);

    /*CV_WRAP virtual int descriptorSize() const;
    CV_WRAP virtual int descriptorType() const;
    CV_WRAP virtual int defaultNorm() const;*/

    //CV_WRAP void write(const String& fileName) const;

    /*CV_WRAP void read(const String& fileName);

    virtual void write(FileStorage&) const CV_OVERRIDE;*/

    // see corresponding cv::Algorithm method
    //CV_WRAP virtual void read(const FileNode&) CV_OVERRIDE;

    //! Return true if detector object is empty
    //CV_WRAP virtual bool empty() const CV_OVERRIDE;
    //CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;

    // see corresponding cv::Algorithm method
    CV_WRAP inline void write(const Ptr<FileStorage>& fs, const String& name = String()) const { Algorithm__::write(fs, name); }
};

Feature2D__::~Feature2D__() {}

void Feature2D__::detect(InputArray image,
    std::vector<KeyPoint>& keypoints,
    InputArray mask)
{

    if (image.empty())
    {
        keypoints.clear();
        return;
    }
    detectAndCompute(image, mask, keypoints, noArray(), false);
}

void Feature2D__::compute(InputArray image,
    std::vector<KeyPoint>& keypoints,
    OutputArray descriptors)
{
    //CV_INSTRUMENT_REGION();

    if (image.empty())
    {
        descriptors.release();
        return;
    }
    detectAndCompute(image, noArray(), keypoints, descriptors, true);
}

void Feature2D__::detectAndCompute(InputArray, InputArray,
    std::vector<KeyPoint>&,
    OutputArray,
    bool)
{
    //CV_INSTRUMENT_REGION();

    CV_Error(Error::StsNotImplemented, "");
}



/////////////////////////////////////////     fast math    ////////////////////////////////////////////

CV_INLINE int cvRound__(double value)
{
#if defined CV_INLINE_ROUND_DBL
    CV_INLINE_ROUND_DBL(value);
#elif ((defined _MSC_VER && defined _M_X64) || (defined __GNUC__ && defined __x86_64__ \
    && defined __SSE2__ && !defined __APPLE__) || CV_SSE2) \
    && !defined(__CUDACC__)
    __m128d t = _mm_set_sd(value);
    return _mm_cvtsd_si32(t);
#elif defined _MSC_VER && defined _M_IX86
    int t;
    __asm
    {
        fld value;
        fistp t;
    }
    return t;
#elif defined CV_ICC || defined __GNUC__
    return (int)(lrint(value));
#else
    /* it's ok if round does not comply with IEEE754 standard;
       the tests should allow +/-1 difference when the tested functions use round */
    return (int)(value + (value >= 0 ? 0.5 : -0.5));
#endif
}

CV_INLINE int cvRound__(float value)
{
#if defined CV_INLINE_ROUND_FLT
    CV_INLINE_ROUND_FLT(value);
#elif ((defined _MSC_VER && defined _M_X64) || (defined __GNUC__ && defined __x86_64__ \
    && defined __SSE2__ && !defined __APPLE__) || CV_SSE2) \
    && !defined(__CUDACC__)
    __m128 t = _mm_set_ss(value);
    return _mm_cvtss_si32(t);
#elif defined _MSC_VER && defined _M_IX86
    int t;
    __asm
    {
        fld value;
        fistp t;
    }
    return t;
#elif defined CV_ICC || defined __GNUC__
    return (int)(lrintf(value));
#else
    /* it's ok if round does not comply with IEEE754 standard;
     the tests should allow +/-1 difference when the tested functions use round */
    return (int)(value + (value >= 0 ? 0.5f : -0.5f));
#endif
}

/** @overload */
CV_INLINE int cvRound__(int value)
{
    return value;
}

CV_INLINE int cvFloor__(float value)
{
#if (defined CV__FASTMATH_ENABLE_GCC_MATH_BUILTINS || defined CV__FASTMATH_ENABLE_CLANG_MATH_BUILTINS) \
    && ( \
        defined(__PPC64__) \
    )
    return __builtin_floorf(value);
#else
    int i = (int)value;
    return i - (i > value);
#endif
}

CV_INLINE int cvCeil__(float value)
{
#if (defined CV__FASTMATH_ENABLE_GCC_MATH_BUILTINS || defined CV__FASTMATH_ENABLE_CLANG_MATH_BUILTINS) \
    && ( \
        defined(__PPC64__) \
    )
    return __builtin_ceilf(value);
#else
    int i = (int)value;
    return i + (i < value);
#endif
}

CV_INLINE int cvCeil__(double value)
{
#if (defined CV__FASTMATH_ENABLE_GCC_MATH_BUILTINS || defined CV__FASTMATH_ENABLE_CLANG_MATH_BUILTINS) \
    && ( \
        defined(__PPC64__) \
    )
    return __builtin_ceil(value);
#else
    int i = (int)value;
    return i + (i < value);
#endif
}

static const float atan2_p1 = 0.9997878412794807f * (float)(180 / CV_PI);
static const float atan2_p3 = -0.3258083974640975f * (float)(180 / CV_PI);
static const float atan2_p5 = 0.1555786518463281f * (float)(180 / CV_PI);
static const float atan2_p7 = -0.04432655554792128f * (float)(180 / CV_PI);
static inline float atan_f32__(float y, float x)
{
    float ax = std::abs(x), ay = std::abs(y);
    float a, c, c2;
    if (ax >= ay)
    {
        c = ay / (ax + (float)DBL_EPSILON);
        c2 = c * c;
        a = (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    else
    {
        c = ax / (ay + (float)DBL_EPSILON);
        c2 = c * c;
        a = 90.f - (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    if (x < 0)
        a = 180.f - a;
    if (y < 0)
        a = 360.f - a;
    return a;
}

float fastAtan2__(float y, float x)
{
    return atan_f32__(y, x);
}




//////////////////////////////////////////////////////////////////////////////////////////

static inline size_t alignSize__(size_t sz, int n)
{
    CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (sz + n - 1) & -n;
}



/////////////////////**************    SIFT   ***************///////////



static const int SIFT_IMG_BORDER_ = 5;

enum InterpolationFlags {

    INTER_NEAREST_ = 0,

    //INTER_LINEAR_ = 1,

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
    parallel_for__(Range(0, static_cast<int>(keypoints.size())), calcDescriptorsComputer(gpyr, keypoints, descriptors, nOctaveLayers, firstOctave));
}

void SIFT_Impl__::detectAndCompute(InputArray _image, InputArray _mask,
    std::vector<KeyPoint>& keypoints,
    OutputArray _descriptors,
    bool useProvidedKeypoints)
{
    //CV_TRACE_FUNCTION();

    int firstOctave = -1, actualNOctaves = 0, actualNLayers = 0;
    Mat image = _image.getMat(), mask = _mask.getMat();

   /*if (image.empty() || image.depth() != CV_8U)
        CV_Error(Error::StsBadArg, "image is empty or has incorrect depth (!=CV_8U)");

    if (!mask.empty() && mask.type() != CV_8UC1)
        CV_Error(Error::StsBadArg, "mask has incorrect type (!=CV_8UC1)");*/

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
        KeyPointsFilter__::removeDuplicatedSorted(keypoints);

        if (nfeatures > 0)
            KeyPointsFilter__::retainBest(keypoints, nfeatures);
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
            KeyPointsFilter__::runByPixelsMask(keypoints, mask);
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
                resize__(src, dst, Size(src.cols / 2, src.rows / 2),
                    0, 0, INTER_NEAREST_);
            }
            else
            {


                
             
                const Mat& src = pyr[o * (nOctaveLayers + 3) + i - 1];

                int sdepth = CV_MAT_DEPTH(src.type());
                //cout << sdepth << endl;

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

    parallel_for__(Range(0, nOctaves * (nOctaveLayers + 2)), buildDoGPyramidComputer(nOctaveLayers, gpyr, dogpyr));
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

            parallel_for__(Range(SIFT_IMG_BORDER_, rows - SIFT_IMG_BORDER_),
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





/////////////////////////         SIFT           //////////////

void Methods::read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1)
/*******************************************************************************
	Read the calibration file from 'filePath', and return projection matices
	of camera0 and camera1 as P0 and P1.

	Arguments:
		filePath -- file path of the calibration file
		P0 -- pointer to projection matrix of camera0
		P1 -- pointer to projection matrix of camera1

*******************************************************************************/
{
	FILE* fp;
	fopen_s(&fp, filePath, "r");
	char* next_token1 = NULL;
	char* next_token2 = NULL;

	*P0 = cv::Mat(3, 4, CV_32F);
	*P1 = cv::Mat(3, 4, CV_32F);

	if (!fp)
	{
		printf("Could not open the calibration file\n");
	}

	int count = 0;
	bool p;
	char content[1024];
	while (fgets(content, 1024, fp))
	{
		char* v = strtok_s(content, " ,", &next_token1);
		while (v)
		{
			if (--count > 0) 
			{
				istringstream os(v);
				float d;
				os >> d;
				if(p)
					P1->at<float>((12 - count) / 4, (12 - count) % 4) = d;
				else
					P0->at<float>((12 - count) / 4, (12 - count) % 4) = d;
			}
			if (!strcmp(v, "P0:"))
			{
				count = 13;
				p = 0;
			}
			else if (!strcmp(v, "P1:"))
			{
				count = 13;
				p = 1;
			}
			 v = strtok_s(NULL, " ,", &next_token1);
		}
	}

	fclose(fp);
}


vector<Mat> Methods::groundTruthTrajectory(const char* filePath, int data_num)
/*******************************************************************************
	Read the ground truth poses data from 'filePath', return Matrices of 
	ground truth poses in a vector.

	Arguments:
		filePath -- file path of the poses data

	Return:
		poses -- a vector of poses in the form of matrix

*******************************************************************************/
{
	vector<Mat> poses;
	FILE* fp;
	fopen_s(&fp, filePath, "r");
	int cols = 12;
	for (int i = 0; i < data_num; i++)
	{
		Mat mat_i = Mat(3, 4, CV_32F);
		for (int j = 0; j < cols; j++)
		{
			fscanf_s(fp, "%e", &mat_i.at<float>(j / 4, j % 4));
		}
		poses.push_back(mat_i);
	}

	fclose(fp);
	return poses;
}


Mat Methods::computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
/***************************************************************************************
	Takes a left and right stereo pair of images and computes the disparity 
	map for the left image. Pass rgb = true if the images are RGB.
    
    Arguments:
		img_left -- image from left camera
		img_right -- image from right camera
    
    Optional Arguments:
		matcher -- (bool) can be 'BM' for StereoBM or 'SGBM' for StereoSGBM matching
		rgb -- (bool) set to true if passing RGB images as input
    
    Returns:
		disp_left -- disparity map for the left camera image

***************************************************************************************/
{
	// Feel free to read OpenCV documentationand tweak these values.These work well
	int sad_window = 6;
	int num_disparities = sad_window * 16;
	int block_size = 11;

	Ptr<StereoMatcher> matcher;
	Mat disp_left;
	
	if (matcher_name == BM)
	{
		matcher = StereoBM::create(num_disparities, block_size);
	}
	else if (matcher_name == SGBM)
	{
		matcher = StereoSGBM::create(0, num_disparities, block_size, 8 * 3 * pow(sad_window, 2), 32 * 3 * pow(sad_window, 2), 0, 0, 0, 0, 0, StereoSGBM::MODE_SGBM_3WAY);
	}

	if (rgb)
	{
		cvtColor(img_left, img_left, COLOR_BGR2GRAY);
		cvtColor(img_right, img_right, COLOR_BGR2GRAY);
	}
	
	printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
	clock_t start = clock();
	if (matcher_name == BM)
	{
		matcher->compute(img_left, img_right, disp_left);
		disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
	}
	else if (matcher_name == SGBM)
	{
		matcher->compute(img_left, img_right, disp_left);
		disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
	}

	clock_t end = clock();
	printf("\tTime to compute disparity map using Stereo%s: %lld ms\n", (matcher_name == BM) ? "BM" : "SGBM", end - start);
	
	int x = 300, y = 1200;
	//printf("\ncompare with python tutorial, disp_left[%d, %d] = %f\n\n", x, y, disp_left.at<float>(x, y));

	return disp_left;
}



void Methods::decompose_Projection_Matrix(Mat p, Mat* k, Mat* r, Mat* t) 
/*************************************************************************************** 
	Shortcut to use cv::decomposeProjectionMatrix(), which only returns k, r, t, and 
	divides t by the scale, then returns them through pointers

	Arguments:
	p -- projection matrix to be decomposed

	Returns (call by address):
	k, r, t -- intrinsic matrix, rotation matrix, and 3D translation vector

***************************************************************************************/
{
	decomposeProjectionMatrix(p, *k, *r, *t);
	
	*t = *t / (t->at<float>(3));
}


Mat Methods::calc_depth_map(Mat disp_left, Mat k_left, Mat t_left, Mat t_right, bool rectified)
/***************************************************************************************
	Calculate depth map using a disparity map, intrinsic camera matrix, and translation
	vectors from camera extrinsic matrices(to calculate baseline).
	Note that default behavior is for rectified projection matrix for right camera.
	If using a regular projection matrix, pass rectified = false to avoid issues.

	Arguments:
		disp_left -- disparity map of left camera
		k_left -- intrinsic matrix for left camera
		t_left -- translation vector for left camera
		t_right -- translation vector for right camera
		rectified-- (bool)set to False if t_right is not from rectified projection 
					matrix

	Returns :
		depth_map -- calculated depth map for left camera

***************************************************************************************/
{
	Mat depth_map = Mat::ones(disp_left.rows, disp_left.cols, CV_32F);
	disp_left.convertTo(disp_left, CV_32F);

	// Get focal length of x axis for left camera
	float f = k_left.at<float>(0, 0);
	
	// Calculate baseline of stereo pair
	float b;
	if (rectified)
		b = t_right.at<float>(0, 0) - t_left.at<float>(0, 0);
	else
		b = t_left.at<float>(0, 0) - t_right.at<float>(0, 0);


	for (int i = 0; i < disp_left.rows; i++)
	{
		for (int j = 0; j < disp_left.cols; j++)
		{
			// Avoid instability and division by zero
			if (disp_left.at<float>(i, j) == 0.0 ||
				disp_left.at<float>(i, j) == -1.0)
				disp_left.at<float>(i, j) = 0.1;

			// Make empty depth map then fill with depth
			depth_map.at<float>(i, j) = f * b / disp_left.at<float>(i, j);
		}
	}

	return depth_map;
}


Mat Methods::stereo_2_depth(Mat img_left, Mat img_right, Mat P0, Mat P1, bool matcher, bool rgb, bool rectified)
/***************************************************************************************
	Takes stereo pair of images and returns a depth map for the left camera.If your 
	projection matrices are not rectified, set rectified = false.

	Arguments:
		img_left -- image of left camera
		img_right -- image of right camera
		P0 -- Projection matrix for the left camera
		P1 -- Projection matrix for the right camera

	Optional Arguments :
		matcher-- (str)can be 'bm' for StereoBM or 'sgbm' for StereoSGBM
		rgb-- (bool)set to True if images passed are RGB.Default is False
		rectified-- (bool)set to False if P1 not rectified to P0.Default is True

	Returns :
		depth -- depth map for left camera

***************************************************************************************/
{
	// Compute disparity map
	Mat disp = computeLeftDisparityMap(img_left,
		img_right,
		matcher,
		rgb);

	// Decompose projection matrices
	Mat k_left, r_left, t_left;
	Mat k_right, r_right, t_right;
	decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
	decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);

	// Calculate depth map for left camera
	Mat depth = calc_depth_map(disp, k_left, t_left, t_right, true);

	return depth;
}



Mat Methods::extract_features(Mat image, int detector, Mat mask, vector<KeyPoint>* kp)
/***************************************************************************************
	Find keypoints and descriptors for the image

	Arguments :
		image -- a grayscale image
		detector-- (bool)can be 'Sift' or 'Orb'
		mask -- (Mat) mask to reduce feature search area to where depth information 
				available.

	Returns :
		kp (call by address) -- list of the extracted keypoints(features) in an image
		des -- list of the keypoint descriptors in an image

***************************************************************************************/
 {

	Ptr<Feature2D> det;
	Mat des; 
	//Ptr<SURF> det_surf;
	
	if (detector == Sift)
	{
		det = SIFT__::create();
		det->Feature2D::detect(image, *kp, mask);
		det->Feature2D::compute(image, *kp, des);

	}
	/*else if (detector == Orb)
	{
		det = ORB__::create();
		det->Feature2D__::detect(image, *kp, mask);
		det->Feature2D__::compute(image, *kp, des);
	}*/


	return des;
}



vector<vector<DMatch>> Methods::match_features(Mat des1, Mat des2, bool matching, int detector, bool sorting, int k)
/***************************************************************************************
	Match features from two images

	Arguments :
		des1 -- list of the keypoint descriptors in the first image
		des2 -- list of the keypoint descriptors in the second image
		matching-- (bool)can be 'BF' for Brute Force or 'FLANN'
		detector-- (int)can be 'Sift or 'Orb'
		sort-- (bool)whether to sort matches by distance.Default is True
		k-- (int)number of neighbors to match to each feature.

	Returns:
		matches -- list of matched features from two images.Each match[i] is k or less 
		matches for the same query descriptor
***************************************************************************************/
{
	vector<vector<DMatch>> matches;


	clock_t start = clock();

	if (matching == BF)
	{
		BFMatcher matcher;
		if (detector == Sift)
			matcher.BFMatcher::create(NORM_L2, false);
		else if (detector == Orb)
			matcher.BFMatcher::create(NORM_HAMMING2, false);
		matcher.BFMatcher::knnMatch(des1, des2, matches, k);
	}
	else if (matching == FLANN)
	{
	
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		matcher->knnMatch(des1, des2, matches, 2);
	}
	clock_t end = clock();

	printf("\tTime to match keypoints using %s: %lld ms\n\n", (matching == BF) ? "BF" : "FLANN", end - start);

	return matches;
}


void Methods::visualize_matches(Mat image1, vector<KeyPoint> kp1, Mat image2, vector<KeyPoint> kp2, vector<vector<DMatch>> match)
/***************************************************************************************
	Visualize corresponding matches in two images

	Arguments :
		image1 -- the first image in a matched image pair
		kp1 -- list of the keypoints in the first image
		image2 -- the second image in a matched image pair
		kp2 -- list of the keypoints in the second image
		match -- list of matched features from the pair of images

	Returns :
		image_matches -- an image showing the corresponding matches on both image1 and 
		image2 or None if you don't use this function

***************************************************************************************/
{
	Mat image_matches;
	drawMatches(image1, kp1, image2, kp2, match, image_matches);
	imshow("image matches", image_matches);
	waitKey();
	destroyWindow("image matches");
	system("cls");
}


vector<vector<DMatch>> Methods::filter_matches_distance(vector<vector<DMatch>> matches, float dist_threshold)
/***************************************************************************************
	Filter matched features from two images by distance between the best matches

	Arguments :
		match -- list of matched features from two images
		dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)

	Returns :
		filtered_match -- list of good matches, satisfying the distance threshold

***************************************************************************************/
{
	vector<vector<DMatch>> filtered_match;
	for (int m = 0; m < matches.size(); m++)
	{

		if (matches[m][0].distance <= dist_threshold * matches[m][1].distance)
		{
			vector<DMatch> match_i;
			match_i.push_back(matches[m][0]);
			filtered_match.push_back(match_i);
		}
	}
	return filtered_match;
}



void Methods::estimate_motion(vector<vector<DMatch>> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat k, Mat depth1, int max_depth,
	Mat &rmat, Mat &tvec, Mat &image1_points,  Mat &image2_points)
/***************************************************************************************
	Estimate camera motion from a pair of subsequent image frames

	Arguments :
		match -- list of matched features from the pair of images
		kp1 -- list of the keypoints in the first image
		kp2 -- list of the keypoints in the second image
		k -- camera intrinsic calibration matrix
		depth1 -- Depth map of the first frame.Set to None to use Essential Matrix 
				decomposition
		max_depth -- Threshold of depth to ignore matched features. 3000 is default

	Returns (call by reference) :
		rmat -- estimated 3x3 rotation matrix
		tvec -- estimated 3x1 translation vector
		image1_points -- matched feature pixel coordinates in the first image.
		image1_points[i] = [u, v]->pixel coordinates of i - th match
		image2_points -- matched feature pixel coordinates in the second image.
		image2_points[i] = [u, v]->pixel coordinates of i - th match
***************************************************************************************/
{
	Mat image1_points__ = Mat(0, 2, CV_32F);
	image1_points = Mat(0, 2, CV_32F);
	Mat image2_points__ = Mat(0, 2, CV_32F);
	image2_points = Mat(0, 2, CV_32F);
	Mat rvec;
	Mat distCoef = Mat::zeros(1, 5, CV_32F);

	for (int m = 0; m < match.size(); m++)
	{
		image1_points__.push_back(kp1[match[m][0].queryIdx].pt);
		image2_points__.push_back(kp2[match[m][0].trainIdx].pt);
	}

	if (!depth1.empty())
	{
		float cx = k.at<float>(0, 2);
		float cy = k.at<float>(1, 2);
		float fx = k.at<float>(0, 0);
		float fy = k.at<float>(1, 1);
		Mat object_points = Mat::zeros(0, 3, CV_32F);

		for (int i = 0; i < image1_points__.rows; i++)
		{

			float u = image1_points__.at<float>(i, 0);
			float v = image1_points__.at<float>(i, 1);
			float z = depth1.at<float>((int)v, (int)u);

			if (z > max_depth)
			{
				continue;
			}


			float x = z * (u - cx) / fx;
			float y = z * (v - cy) / fy;


			Mat vec = Mat(1, 3, CV_32F);
			vec.at<float>(0, 0) = x;
			vec.at<float>(0, 1) = y;
			vec.at<float>(0, 2) = z;


			object_points.push_back(vec);
			image1_points.push_back(image1_points__.row(i));
			image2_points.push_back(image2_points__.row(i));
		}
		printf("max: %d\n", max(object_points.checkVector(3, CV_32F), object_points.checkVector(3, CV_64F)));
		cv::solvePnPRansac(object_points, image2_points, k, distCoef, rvec, tvec, 
			false, 100, 8.0, 0.99, noArray(), SOLVEPNP_ITERATIVE);


		rmat = Mat::eye(3, 3, CV_32F);
		Rodrigues(rvec, rmat);
	}
}


vector<Mat> Methods::visual_odometry(Dataset_Handler handler, int detector, bool matching, 
	float filter_match_distance, bool stereo_matcher, int subset, Mat mask)
/***************************************************************************************
	Function to perform visual odometry on a sequence from the KITTI visual odometry 
	dataset.
	Takes as input a Dataset_Handler object and optional parameters.

	Arguments:
		handler -- Dataset_Handler object instance
		detector -- (str) can be 'Sift' or 'Orb'.
		matching -- (str) can be 'BF' for Brute Force or 'FLANN'. 
		filter_match_distance -- (float) value for ratio test on matched features. 
								Default is None.
		stereo_matcher -- (str) can be 'BM' (faster) or 'SGBM' (more accurate). 
		mask -- (array) mask to reduce feature search area to where depth information 
					available.
		subset -- (int) number of frames to compute. Defaults to None to compute 
						all frames.

	Returns:
		trajectory -- Array of shape Nx3x4 of estimated poses of vehicle for each 
					computed frame.
***************************************************************************************/
{
	int num_frames;

	printf("Generating disparities with Stereo %s\n", stereo_matcher ? "SGBM" : "BM");
	printf("Detecting features with %s and matching with %s\n", (detector == Sift) ? "SIFT" : (detector == Orb) ? "ORB" : "SURF",
		matching ? "BF" : "FLANN");
		
	if (filter_match_distance)
		printf("Filtering feature matches at threshold of %f * distance\n", filter_match_distance);
		
	if (subset)
		num_frames = subset;
	else
		num_frames = handler.num_frames;

	Mat T_tot = Mat::eye(4, 4, CV_64F);


	vector<Mat> trajectory(num_frames);
	Rect rect(0, 0, 4, 3);
	T_tot(rect).copyTo(trajectory[0]);

	int imwidth = handler.imwidth;
	int imheight = handler.imheight;

	Mat k_left, r_left, t_left;
	decompose_Projection_Matrix(handler.P0, &k_left, &r_left, &t_left);

	Mat image_plus1 = imread(handler.left_image_files[0], IMREAD_GRAYSCALE);
	Mat image_left, image_right;
	Mat depth;

	

	for (int i = 0; i < num_frames - 1; i++)
	{
		printf("Computing frame %d\n", i + 1);
		clock_t start = clock();
		image_left = image_plus1;
		image_right = imread(handler.right_image_files[i], IMREAD_GRAYSCALE);
		image_plus1 = imread(handler.left_image_files[i+1], IMREAD_GRAYSCALE);

		depth = stereo_2_depth(image_left, image_right, handler.P0, handler.P1, stereo_matcher, false, true);


		vector<KeyPoint> kp0, kp1;
		/*test extract;
		Mat des0 = extract.extract_features(image_left, detector, mask, &kp0);
		Mat des1 = extract.extract_features(image_plus1, detector, mask, &kp1);*/
		Mat des0 = extract_features(image_left, detector, mask, &kp0);

		cout << "We are writing!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		/*Mat testdes0 =
			extract_features(image_left, detector, mask, &kp0);
		Mat testdes1;

		ofstream fileA;
		fileA.open("D:\\opencv_project\\output1\\matfiletest.txt", ios::trunc);
		fileA << testdes0.cols << "\n";
		fileA << testdes0.rows << "\n";
		fileA << *testdes0.data << "\n";
		
		cout << testdes0 << endl;
		fileA.close();

		ifstream fileI;
		fileI.open("D:\\opencv_project\\output1\\matfiletest.txt");
		fileI >> testdes1.cols >> testdes1.rows;
		fileI >> *testdes1.data;
		fileI.close();*/



		cout << "We close writing!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

		Mat des1 = extract_features(image_plus1, detector, mask, &kp1);


		

		vector<vector<DMatch>> matches_unfilt = match_features(des0, des1, matching, detector, false, 2);

		vector<vector<DMatch>> matches;
		if (filter_match_distance)
			matches = filter_matches_distance(matches_unfilt, filter_match_distance);
		else
			matches = matches_unfilt;

		Mat rmat, tvec, img1_points, img2_points;
		estimate_motion(matches, kp0, kp1, k_left, depth, 3000, rmat, tvec, img1_points, img2_points);

		Mat T_mat;
		Mat I4 = Mat::eye(4, 4, CV_64F);
		hconcat(rmat, tvec, T_mat);
		vconcat(T_mat, I4.row(3), T_mat);
		T_tot = T_tot * T_mat.inv();
		T_tot(rect).copyTo(trajectory[i+1]);
		clock_t end = clock();

		printf("Time to compute frame %d: %lld ms\n\n", i + 1, end - start);
	}
	
	return trajectory;
}
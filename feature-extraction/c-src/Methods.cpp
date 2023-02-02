#include "Methods.h"
#include <fstream>
#include <stdlib.h>

#include "resize1.h"
#include "threshold__.h"
#include "copyMakeBorder__.h"
#include "GaussianBlur__.h"
#include "KeyPointsFilter__.h"
#include "AutoBuffer__.h"
#include "RNG__.h"
#include "BufferArea__.h"
#include "FAST__.h"
#include "Point__.h"
//#include "SIFT__.h"
//#include "createInitialImage__.h"
//#include "buildDoGPyramidComputer.h"
//#include "findScaleSpaceExtremaComputer.h"
//#include "calcDescriptorsComputer.h"

//#include "TLSDataAccumulator.h"
//#include "parallel_for__.h"
//#include "Vec3f__.h"






using namespace std;
using namespace cv;
using namespace utils;
//#pragma warning( disable : 4996 )

#define CV_CN_MAX     512
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)

#define CV_8U   0
#define CV_8S   1
#define CV_16U  2
#define CV_16S  3
#define CV_32S  4
#define CV_32F  5
#define CV_64F  6
#define CV_16F  7

#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)

#define CV_MAKETYPE_(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))

#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)

#  define CV_OVERRIDE override
#    define CV_INLINE static inline
#  define CV_ENABLE_UNROLLED 1

typedef unsigned char uchar;
typedef unsigned __int64 size_t;

#define CV_PI   3.1415926535897932384626433832795






class Algorithm__;
//template<typename _Tp, typename _EnumTp = void> struct ParamType {};

class Algorithm__
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
     void write(const Ptr<FileStorage>& fs, const String& name = String()) const;

    /** @brief Reads algorithm parameters from a file storage
    */
     virtual void read(const FileNode& fn) { CV_UNUSED(fn); }

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
        //CV_Assert(fs.isOpened());
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
     virtual void save(const String& filename) const;

    /** Returns the algorithm string identifier.
    This string is used as top level xml/yml node tag when the object is saved to a file or string. */
     virtual String getDefaultName() const;

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


class Feature2D__ : public virtual Algorithm__
{
public:
    virtual ~Feature2D__();

    
     virtual void detect(InputArray image,
         std::vector<KeyPoint>& keypoints,
        InputArray mask = noArray());

    
    /*CV_WRAP virtual void detect(InputArrayOfArrays images,
        CV_OUT std::vector<std::vector<KeyPoint> >& keypoints,
        InputArrayOfArrays masks = noArray());*/

   
     virtual void compute(InputArray image,
          std::vector<KeyPoint>& keypoints,
        OutputArray descriptors);

    
    /*CV_WRAP virtual void compute(InputArrayOfArrays images,
        CV_OUT CV_IN_OUT std::vector<std::vector<KeyPoint> >& keypoints,
        OutputArrayOfArrays descriptors);*/

    /** Detects keypoints and computes the descriptors */
     virtual void detectAndCompute(InputArray image, InputArray mask,
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
     inline void write(const Ptr<FileStorage>& fs, const String& name = String()) const { Algorithm__::write(fs, name); }
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

    //CV_Error(Error::StsNotImplemented, "");
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

/////////////////////////////  FastFeatureDetector  ///////////////////////////////////



class  FastFeatureDetector__ : public Feature2D__
{
public:
    enum DetectorType
    {
        TYPE_5_8 = 0, TYPE_7_12 = 1, TYPE_9_16 = 2
    };
    enum
    {
        THRESHOLD = 10000, NONMAX_SUPPRESSION = 10001, FAST_N = 10002
    };


     static Ptr<FastFeatureDetector__> create(int threshold = 10,
        bool nonmaxSuppression = true,
        FastFeatureDetector__::DetectorType type = FastFeatureDetector__::TYPE_9_16);

   /* CV_WRAP virtual void setThreshold(int threshold) = 0;
    CV_WRAP virtual int getThreshold() const = 0;

    CV_WRAP virtual void setNonmaxSuppression(bool f) = 0;
    CV_WRAP virtual bool getNonmaxSuppression() const = 0;

    CV_WRAP virtual void setType(FastFeatureDetector__::DetectorType type) = 0;
    CV_WRAP virtual FastFeatureDetector__::DetectorType getType() const = 0;*/
    //CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;
};
/*
static inline int hal_FAST(cv::Mat& src, std::vector<KeyPoint>& keypoints, int threshold, bool nonmax_suppression, FastFeatureDetector__::DetectorType type)
{
    if (threshold > 20)
        return CV_HAL_ERROR_NOT_IMPLEMENTED;

    cv::Mat scores(src.size(), src.type());

    int error = 1;

    if (error != CV_HAL_ERROR_OK)
        return error;

    cv::Mat suppressedScores(src.size(), src.type());

    if (nonmax_suppression)
    {
        error = 1;

        if (error != CV_HAL_ERROR_OK)
            return error;
    }
    else
    {
        suppressedScores = scores;
    }

    if (!threshold && nonmax_suppression) threshold = 1;

    cv::KeyPoint kpt(0, 0, 7.f, -1, 0);

    unsigned uthreshold = (unsigned)threshold;

    int ofs = 3;

    int stride = (int)suppressedScores.step;
    const unsigned char* pscore = suppressedScores.data;

    keypoints.clear();

    for (int y = ofs; y + ofs < suppressedScores.rows; ++y)
    {
        kpt.pt.y = (float)(y);
        for (int x = ofs; x + ofs < suppressedScores.cols; ++x)
        {
            unsigned score = pscore[y * stride + x];
            if (score > uthreshold)
            {
                kpt.pt.x = (float)(x);
                kpt.response = (nonmax_suppression != 0) ? (float)((int)score - 1) : 0.f;
                keypoints.push_back(kpt);
            }
        }
    }

    return CV_HAL_ERROR_OK;
}


#define CALL_HAL(name, fun, ...) \
{                                           \
    int res = __CV_EXPAND(fun(__VA_ARGS__)); \
    if (res == CV_HAL_ERROR_OK) \
        return; \
    else if (res != CV_HAL_ERROR_NOT_IMPLEMENTED) \
        CV_Error_(cv::Error::StsInternal, \
            ("HAL implementation " CVAUX_STR(name) " ==> " CVAUX_STR(fun) " returned %d (0x%08x)", res, res)); \
}
inline int hal_ni_FAST(const uchar* src_data, size_t src_step, int width, int height, uchar* keypoints_data, size_t* keypoints_count, int threshold, bool nonmax_suppression, int /*cv::FastFeatureDetector::DetectorType type) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }

//! @cond IGNORED
#define cv_hal_FAST hal_ni_FAST
*/

/// ////////////////////

///


class FastFeatureDetector_Impl__ CV_FINAL : public FastFeatureDetector__
{
public:
    FastFeatureDetector_Impl__(int _threshold, bool _nonmaxSuppression, FastFeatureDetector__::DetectorType _type)
        : threshold(_threshold), nonmaxSuppression(_nonmaxSuppression), type(_type)
    {
        cout << "IMPL";
    }

    void detect(InputArray _image, std::vector<KeyPoint>& keypoints, InputArray _mask) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();
        cout << "detect" << endl;

        if (_image.empty())
        {
            keypoints.clear();
            return;
        }

        Mat mask = _mask.getMat(), grayImage;
        UMat ugrayImage;
        _InputArray gray = _image;
        /*cout << _image.type() << endl;
        if (_image.type() != CV_8U)
        {
            _OutputArray ogray = _image.isUMat() ? _OutputArray(ugrayImage) : _OutputArray(grayImage);
            cvtColor(_image, ogray, COLOR_BGR2GRAY);
            gray = ogray;
        }*/
        //cout << type << endl;
        FAST__(gray, keypoints, threshold, nonmaxSuppression);
        KeyPointsFilter__::runByPixelsMask(keypoints, mask);
    }

    /*void set(int prop, double value)
    {
        if (prop == THRESHOLD)
            threshold = cvRound(value);
        else if (prop == NONMAX_SUPPRESSION)
            nonmaxSuppression = value != 0;
        else if (prop == FAST_N)
            type = static_cast<FastFeatureDetector__::DetectorType>(cvRound(value));
        else
            CV_Error(Error::StsBadArg, "");
    }

    double get(int prop) const
    {
        if (prop == THRESHOLD)
            return threshold;
        if (prop == NONMAX_SUPPRESSION)
            return nonmaxSuppression;
        if (prop == FAST_N)
            return static_cast<int>(type);
        CV_Error(Error::StsBadArg, "");
        return 0;
    }*/

    /*void setThreshold(int threshold_) CV_OVERRIDE { threshold = threshold_; }
    int getThreshold() const CV_OVERRIDE { return threshold; }

    void setNonmaxSuppression(bool f) CV_OVERRIDE { nonmaxSuppression = f; }
    bool getNonmaxSuppression() const CV_OVERRIDE { return nonmaxSuppression; }

    void setType(FastFeatureDetector__::DetectorType type_) { type = type_; }
    FastFeatureDetector__::DetectorType getType() const { return type; }*/

    int threshold;
    bool nonmaxSuppression;
    FastFeatureDetector__::DetectorType type;
};

Ptr<FastFeatureDetector__> FastFeatureDetector__::create(int threshold, bool nonmaxSuppression, FastFeatureDetector__::DetectorType type)
{
    return makePtr<FastFeatureDetector_Impl__>(threshold, nonmaxSuppression, type);
}
///////////////////////////////////////////////////////////////////////////////////////////////////


class CV_EXPORTS_W ORB__ : public Feature2D__
{
public:
    enum ScoreType { HARRIS_SCORE = 0, FAST_SCORE = 1 };
    static const int kBytes = 32;


    CV_WRAP static Ptr<ORB__> create(int nfeatures = 500, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
        int firstLevel = 0, int WTA_K = 2, ORB__::ScoreType scoreType = ORB__::HARRIS_SCORE, int patchSize = 31, int fastThreshold = 20);

    /* CV_WRAP virtual void setMaxFeatures(int maxFeatures) = 0;
     CV_WRAP virtual int getMaxFeatures() const = 0;

     CV_WRAP virtual void setScaleFactor(double scaleFactor) = 0;
     CV_WRAP virtual double getScaleFactor() const = 0;

     CV_WRAP virtual void setNLevels(int nlevels) = 0;
     CV_WRAP virtual int getNLevels() const = 0;

     CV_WRAP virtual void setEdgeThreshold(int edgeThreshold) = 0;
     CV_WRAP virtual int getEdgeThreshold() const = 0;

     CV_WRAP virtual void setFirstLevel(int firstLevel) = 0;
     CV_WRAP virtual int getFirstLevel() const = 0;

     CV_WRAP virtual void setWTA_K(int wta_k) = 0;
     CV_WRAP virtual int getWTA_K() const = 0;

     CV_WRAP virtual void setScoreType(ORB__::ScoreType scoreType) = 0;
     CV_WRAP virtual ORB__::ScoreType getScoreType() const = 0;

     CV_WRAP virtual void setPatchSize(int patchSize) = 0;
     CV_WRAP virtual int getPatchSize() const = 0;

     CV_WRAP virtual void setFastThreshold(int fastThreshold) = 0;
     CV_WRAP virtual int getFastThreshold() const = 0;*/
    // CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;
};



class ORB_Impl__ CV_FINAL : public ORB__
{
public:
    explicit ORB_Impl__(int _nfeatures, float _scaleFactor, int _nlevels, int _edgeThreshold,
        int _firstLevel, int _WTA_K, ORB__::ScoreType _scoreType, int _patchSize, int _fastThreshold) :
        nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
        edgeThreshold(_edgeThreshold), firstLevel(_firstLevel), wta_k(_WTA_K),
        scoreType(_scoreType), patchSize(_patchSize), fastThreshold(_fastThreshold)
    {
        cout << "ORB_Impl__" << endl;
    }

   /* void setMaxFeatures(int maxFeatures) CV_OVERRIDE { nfeatures = maxFeatures; }
    int getMaxFeatures() const CV_OVERRIDE { return nfeatures; }

    void setScaleFactor(double scaleFactor_) CV_OVERRIDE { scaleFactor = scaleFactor_; }
    double getScaleFactor() const CV_OVERRIDE { return scaleFactor; }

    void setNLevels(int nlevels_) CV_OVERRIDE { nlevels = nlevels_; }
    int getNLevels() const CV_OVERRIDE { return nlevels; }

    void setEdgeThreshold(int edgeThreshold_) CV_OVERRIDE { edgeThreshold = edgeThreshold_; }
    int getEdgeThreshold() const CV_OVERRIDE { return edgeThreshold; }

    void setFirstLevel(int firstLevel_) CV_OVERRIDE { CV_Assert(firstLevel_ >= 0);  firstLevel = firstLevel_; }
    int getFirstLevel() const CV_OVERRIDE { return firstLevel; }

    void setWTA_K(int wta_k_) CV_OVERRIDE { wta_k = wta_k_; }
    int getWTA_K() const CV_OVERRIDE { return wta_k; }

    void setScoreType(ORB__::ScoreType scoreType_) CV_OVERRIDE { scoreType = scoreType_; }
    ORB__::ScoreType getScoreType() const CV_OVERRIDE { return scoreType; }

    void setPatchSize(int patchSize_) CV_OVERRIDE { patchSize = patchSize_; }
    int getPatchSize() const CV_OVERRIDE { return patchSize; }

    void setFastThreshold(int fastThreshold_) CV_OVERRIDE { fastThreshold = fastThreshold_; }
    int getFastThreshold() const CV_OVERRIDE { return fastThreshold; }*/

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // returns the descriptor type
    int descriptorType() const;
    // returns the default norm type
    int defaultNorm() const;

    // Compute the ORB_Impl features and descriptors on an image
    void detectAndCompute(InputArray image, InputArray mask, std::vector<KeyPoint>& keypoints,
        OutputArray descriptors, bool useProvidedKeypoints = false);

protected:

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int edgeThreshold;
    int firstLevel;
    int wta_k;
    ORB__::ScoreType scoreType;
    int patchSize;
    int fastThreshold;
};

int ORB_Impl__::descriptorSize() const
{
    return kBytes;
}

int ORB_Impl__::descriptorType() const
{
    return 0;
}

int ORB_Impl__::defaultNorm() const
{
    switch (wta_k)
    {
    case 2:
        return NORM_HAMMING;
    case 3:
    case 4:
        return NORM_HAMMING2;
    default:
        return -1;
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
    size_t size_t_step = img.step;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void
computeOrbDescriptors(const Mat& imagePyramid, const std::vector<Rect>& layerInfo,
    const std::vector<float>& layerScale, std::vector<KeyPoint>& keypoints,
    Mat& descriptors, const std::vector<Point__>& _pattern, int dsize, int wta_k)
{
    int step = (int)imagePyramid.step;
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
        /*else if (wta_k == 3)
        {
            for (i = 0; i < dsize; ++i, pattern += 12)
            {
                int t0, t1, t2, val;
                t0 = GET_VALUE(0); t1 = GET_VALUE(1); t2 = GET_VALUE(2);
                val = t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0);

                t0 = GET_VALUE(3); t1 = GET_VALUE(4); t2 = GET_VALUE(5);
                val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 2;

                t0 = GET_VALUE(6); t1 = GET_VALUE(7); t2 = GET_VALUE(8);
                val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 4;

                t0 = GET_VALUE(9); t1 = GET_VALUE(10); t2 = GET_VALUE(11);
                val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 6;

                desc[i] = (uchar)val;
            }
        }
        else if (wta_k == 4)
        {
            for (i = 0; i < dsize; ++i, pattern += 16)
            {
                int t0, t1, t2, t3, u, v, k, val;
                t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                t2 = GET_VALUE(2); t3 = GET_VALUE(3);
                u = 0, v = 2;
                if (t1 > t0) t0 = t1, u = 1;
                if (t3 > t2) t2 = t3, v = 3;
                k = t0 > t2 ? u : v;
                val = k;

                t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                t2 = GET_VALUE(6); t3 = GET_VALUE(7);
                u = 0, v = 2;
                if (t1 > t0) t0 = t1, u = 1;
                if (t3 > t2) t2 = t3, v = 3;
                k = t0 > t2 ? u : v;
                val |= k << 2;

                t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                t2 = GET_VALUE(10); t3 = GET_VALUE(11);
                u = 0, v = 2;
                if (t1 > t0) t0 = t1, u = 1;
                if (t3 > t2) t2 = t3, v = 3;
                k = t0 > t2 ? u : v;
                val |= k << 4;

                t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                t2 = GET_VALUE(14); t3 = GET_VALUE(15);
                u = 0, v = 2;
                if (t1 > t0) t0 = t1, u = 1;
                if (t3 > t2) t2 = t3, v = 3;
                k = t0 > t2 ? u : v;
                val |= k << 6;

                desc[i] = (uchar)val;
            }
        }*/
        else
            ;// CV_Error(Error::StsBadSize, "Wrong wta_k. It can be only 2, 3 or 4.");
#undef GET_VALUE
    }
}


/*
static void initializeOrbPattern(const Point__* pattern0, std::vector<Point__>& pattern, int ntuples, int tupleSize, int poolSize)
{
    RNG rng(0x12345678);
    int i, k, k1;
    pattern.resize(ntuples * tupleSize);

    for (i = 0; i < ntuples; i++)
    {
        for (k = 0; k < tupleSize; k++)
        {
            for (;;)
            {
                int idx = rng.uniform(0, poolSize);
                Point__ pt = pattern0[idx];
                for (k1 = 0; k1 < k; k1++)
                    if (pattern[tupleSize * i + k1] == pt)
                        break;
                if (k1 == k)
                {
                    pattern[tupleSize * i + k] = pt;
                    break;
                }
            }
        }
    }
}*/

static int bit_pattern_31_[256 * 4] =
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};


static void makeRandomPattern(int patchSize, Point__* pattern, int npoints)
{
    RNG rng(0x34985739); // we always start with a fixed seed,
    // to make patterns the same on each run
    for (int i = 0; i < npoints; i++)
    {
        pattern[i].x = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
        pattern[i].y = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
    }
}


static inline float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}






/** Compute the ORB_Impl keypoints on an image
 * @param image_pyramid the image pyramid to compute the features and descriptors on
 * @param mask_pyramid the masks to apply at every level
 * @param keypoints the resulting keypoints, clustered per level
 */
static void computeKeyPoints(const Mat& imagePyramid,
    const UMat& uimagePyramid,
    const Mat& maskPyramid,
    const std::vector<Rect>& layerInfo,
    const UMat& ulayerInfo,
    const std::vector<float>& layerScale,
    std::vector<KeyPoint>& allKeypoints,
    int nfeatures, double scaleFactor,
    int edgeThreshold, int patchSize, ORB__::ScoreType scoreType,
    bool useOCL, int fastThreshold)
{
    /*#ifndef HAVE_OPENCL
        CV_UNUSED(uimagePyramid); CV_UNUSED(ulayerInfo); CV_UNUSED(useOCL);
    #endif*/

    int i, nkeypoints, level, nlevels = (int)layerInfo.size(); //金字塔層數  
    std::vector<int> nfeaturesPerLevel(nlevels);

    // fill the extractors and descriptors for the corresponding scales
    float factor = (float)(1.0 / scaleFactor);
    float ndesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)std::pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for (level = 0; level < nlevels - 1; level++) //對每層圖像上分配相應角點數
    {
        nfeaturesPerLevel[level] = cvRound(ndesiredFeaturesPerScale);
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
        umax[v] = cvRound__(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

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

    for (level = 0; level < nlevels; level++)
    {
        int featuresNum = nfeaturesPerLevel[level];
        Mat img = imagePyramid(layerInfo[level]);
        Mat mask = maskPyramid.empty() ? Mat() : maskPyramid(layerInfo[level]);

        // Detect FAST features, 20 is a good threshold
        {
            Ptr<FastFeatureDetector__> fd = FastFeatureDetector__::create(fastThreshold, true);
            fd->detect(img, keypoints, mask); //Fast角點檢測  
        }

        // Remove keypoints very close to the border
        KeyPointsFilter__::runByImageBorder(keypoints, img.size(), edgeThreshold); //去除鄰近邊界的點  

        // Keep more points than necessary as FAST does not give amazing corners
        KeyPointsFilter__::retainBest(keypoints, scoreType == ORB_Impl__::HARRIS_SCORE ? 2 * featuresNum : featuresNum);
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

    // std::vector<Vec3i> ukeypoints_buf;

    nkeypoints = (int)allKeypoints.size();
    if (nkeypoints == 0)
    {
        return;
    }
    Mat responses;
    //UMat ukeypoints, uresponses(1, nkeypoints, CV_32F);

    // Select best features using the Harris cornerness (better scoring than FAST)
    if (scoreType == ORB_Impl__::HARRIS_SCORE)
    {
        /*#ifdef HAVE_OPENCL
                if (useOCL)
                {
                    uploadORBKeypoints(allKeypoints, ukeypoints_buf, ukeypoints);
                    useOCL = ocl_HarrisResponses(uimagePyramid, ulayerInfo, ukeypoints,
                        uresponses, nkeypoints, 7, HARRIS_K);
                    if (useOCL)
                    {
                        CV_IMPL_ADD(CV_IMPL_OCL);
                        uresponses.copyTo(responses);
                        for (i = 0; i < nkeypoints; i++)
                            allKeypoints[i].response = responses.at<float>(i);
                    }
                }

                if (!useOCL)
        #endif*/
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
    }

    nkeypoints = (int)allKeypoints.size();

    /*#ifdef HAVE_OPENCL
        if (useOCL)
        {
            UMat uumax;
            if (useOCL)
                copyVectorToUMat(umax, uumax);

            uploadORBKeypoints(allKeypoints, ukeypoints_buf, ukeypoints);
            useOCL = ocl_ICAngles(uimagePyramid, ulayerInfo, ukeypoints, uresponses, uumax,
                nkeypoints, halfPatchSize);

            if (useOCL)
            {
                CV_IMPL_ADD(CV_IMPL_OCL);
                uresponses.copyTo(responses);
                for (i = 0; i < nkeypoints; i++)
                    allKeypoints[i].angle = responses.at<float>(i);
            }
        }

        if (!useOCL)
    #endif*/
    {   // Process each keypoint  爲每個角點計算主方向，質心法
        ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize);
    }

    for (i = 0; i < nkeypoints; i++)
    {
        float scale = layerScale[allKeypoints[i].octave];
        allKeypoints[i].pt *= scale;
    }
}





//////////////////////////////////////////////////////////////////////////////////////////

static inline size_t alignSize__(size_t sz, int n)
{
    CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (sz + n - 1) & -n;
}



////////////////////////////////////////////////////////////////////////////////////////

void ORB_Impl__::detectAndCompute(InputArray _image, InputArray _mask,
    std::vector<KeyPoint>& keypoints,
    OutputArray _descriptors, bool useProvidedKeypoints)
{
    //CV_INSTRUMENT_REGION();

   // CV_Assert(patchSize >= 2);

    bool do_keypoints = !useProvidedKeypoints;
    bool do_descriptors = _descriptors.needed();

    if ((!do_keypoints && !do_descriptors) || _image.empty())
        return;

    //ROI handling
    const int HARRIS_BLOCK_SIZE = 9; //Harris角點響應需要的邊界大小  
    int halfPatchSize = patchSize / 2; //鄰域半徑  
    // sqrt(2.0) is for handling patch rotation
    int descPatchSize = cvCeil__(halfPatchSize * sqrt(2.0));
    int border = max(edgeThreshold, max(descPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1; //採用最大的邊界

    /*#ifdef HAVE_OPENCL
        bool useOCL = ocl::isOpenCLActivated() && OCL_FORCE_CHECK(_image.isUMat() || _descriptors.isUMat());
    #else*/
    bool useOCL = false;
    //#endif

    Mat image = _image.getMat(), mask = _mask.getMat();
    /*if (image.type() != CV_8UC1)
        cvtColor(_image, image, COLOR_BGR2GRAY);  //轉灰度圖  */

    int i, level, nLevels = this->nlevels, nkeypoints = (int)keypoints.size(); //金字塔層數  
    bool sortedByLevel = true;

    if (!do_keypoints) //不做特徵點檢測  
    {
        // if we have pre-computed keypoints, they may use more levels than it is set in parameters
        // !!!TODO!!! implement more correct method, independent from the used keypoint detector.
        // Namely, the detector should provide correct size of each keypoint. Based on the keypoint size
        // and the algorithm used (i.e. BRIEF, running on 31x31 patches) we should compute the approximate
        // scale-factor that we need to apply. Then we should cluster all the computed scale-factors and
        // for each cluster compute the corresponding image.
        //
        // In short, ultimately the descriptor should
        // ignore octave parameter and deal only with the keypoint size.
        nLevels = 0;
        for (i = 0; i < nkeypoints; i++)
        {
            level = keypoints[i].octave;
           // CV_Assert(level >= 0);
            if (i > 0 && level < keypoints[i - 1].octave)
                sortedByLevel = false;
            nLevels = max(nLevels, level);
        }
        nLevels++;
    }

    std::vector<Rect> layerInfo(nLevels);
    std::vector<int> layerOfs(nLevels);
    std::vector<float> layerScale(nLevels);
    Mat imagePyramid, maskPyramid;  //創建尺度金字塔圖像  
    UMat uimagePyramid, ulayerInfo;

    float level0_inv_scale = 1.0f / getScale(0, firstLevel, scaleFactor);
    size_t level0_width = (size_t)cvRound__(image.cols * level0_inv_scale);
    size_t level0_height = (size_t)cvRound__(image.rows * level0_inv_scale);
    Size bufSize((int)alignSize__(level0_width + border * 2, 16), 0);  // TODO change alignment to 64

    int level_dy = (int)level0_height + border * 2;
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

        Rect linfo(level_ofs.x + border, level_ofs.y + border, sz.width, sz.height);
        layerInfo[level] = linfo;
        layerOfs[level] = linfo.y * bufSize.width + linfo.x;
        level_ofs.x += wholeSize.width;
    }
    bufSize.height = level_ofs.y + level_dy;

    imagePyramid.create(bufSize, CV_8U);
    if (!mask.empty())
        maskPyramid.create(bufSize, CV_8U);

    Mat prevImg = image, prevMask = mask;

    // Pre-compute the scale pyramids
    for (level = 0; level < nLevels; ++level)
    {
        Rect linfo = layerInfo[level];
        Size sz(linfo.width, linfo.height);
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        Rect wholeLinfo = Rect(linfo.x - border, linfo.y - border, wholeSize.width, wholeSize.height);
        Mat extImg = imagePyramid(wholeLinfo), extMask;
        Mat currImg = extImg(Rect(border, border, sz.width, sz.height)), currMask;

        if (!mask.empty())
        {
            extMask = maskPyramid(wholeLinfo);
            currMask = extMask(Rect(border, border, sz.width, sz.height));
        }

        // Compute the resized image
        if (level != firstLevel)  //得到金字塔每層的圖像  
        {
            resize__(prevImg, currImg, sz, 0, 0, INTER_LINEAR_EXACT);
            //cout << "currImgR : " << currImg.rows << " currImgC : " << currImg.cols << "currImgD : " << currImg.dims << endl;
            //cout << "size : " << *prevImg.size.p << endl;
            //cout << "data : " << *prevImg.data << endl;


            if (!mask.empty())
            {
                //cout << "!mask.empty()" << endl;
                resize__(prevMask, currMask, sz, 0, 0, INTER_LINEAR_EXACT);
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
            if (!mask.empty())
                copyMakeBorder__(mask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);
        }
        if (level > firstLevel)
        {
            prevImg = currImg;
            prevMask = currMask;
        }
    }
    /*
    if (useOCL)
        copyVectorToUMat(layerOfs, ulayerInfo);
    */

    if (do_keypoints) //提取角點  
    {
        if (useOCL)
            imagePyramid.copyTo(uimagePyramid);

        // Get keypoints, those will be far enough from the border that no check will be required for the descriptor
        computeKeyPoints(imagePyramid, uimagePyramid, maskPyramid,
            layerInfo, ulayerInfo, layerScale, keypoints,
            nfeatures, scaleFactor, edgeThreshold, patchSize, scoreType, useOCL, fastThreshold);
        //cout << "imagePyramid : " << imagePyramid.rows << " imagePyramidC : " << imagePyramid.cols << "imagePyramidD : " << imagePyramid.dims << endl;
        //cout << "_descriptors : " << *_descriptors.data << endl;
    }
    else
    {
        KeyPointsFilter__::runByImageBorder(keypoints, image.size(), edgeThreshold);

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
        int dsize = descriptorSize();

        nkeypoints = (int)keypoints.size();
        if (nkeypoints == 0)
        {
            _descriptors.release();
            return;
        }

        _descriptors.create(nkeypoints, dsize, CV_8U);
        std::vector<Point__> pattern;

        const int npoints = 512;
        Point__ patternbuf[npoints];
        const Point__* pattern0 = (const Point__*)bit_pattern_31_;

        if (patchSize != 31)
        {
            pattern0 = patternbuf;
            makeRandomPattern(patchSize, patternbuf, npoints);
        }

       // CV_Assert(wta_k == 2 || wta_k == 3 || wta_k == 4);
        //cout << "wta_k: " << wta_k << endl; 2
        if (wta_k == 2)
            std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));
        /*else
        {
            int ntuples = descriptorSize() * 4;
            initializeOrbPattern(pattern0, pattern, ntuples, wta_k, npoints);
        }*/

        for (level = 0; level < nLevels; level++)
        {
            // preprocess the resized image
            Mat workingMat = imagePyramid(layerInfo[level]);

            //boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
            GaussianBlur__(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            /*int sdepth = CV_MAT_DEPTH(workingMat.type());
             sdepth */
        }

        /*#ifdef HAVE_OPENCL
                if (useOCL)
                {
                    imagePyramid.copyTo(uimagePyramid);
                    std::vector<Vec4i> kptbuf;
                    UMat ukeypoints, upattern;
                    copyVectorToUMat(pattern, upattern);
                    uploadORBKeypoints(keypoints, layerScale, kptbuf, ukeypoints);

                    UMat udescriptors = _descriptors.getUMat();
                    useOCL = ocl_computeOrbDescriptors(uimagePyramid, ulayerInfo,
                        ukeypoints, udescriptors, upattern,
                        nkeypoints, dsize, wta_k);
                    if (useOCL)
                    {
                        CV_IMPL_ADD(CV_IMPL_OCL);
                    }
                }

                if (!useOCL)
        #endif*/
        {
            Mat descriptors = _descriptors.getMat();
            computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
                keypoints, descriptors, pattern, dsize, wta_k);
            
        }
    }
   /* for (int i = 0; i < keypoints.size(); i++)
    {
        cout << "x : " << keypoints[i].pt.x << " " << " y : " << keypoints[i].pt.y << endl;
    }*/
    cout << "s : " << keypoints.size() << endl;
   
}



Ptr<ORB__> ORB__::create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold,
    int firstLevel, int wta_k, ORB__::ScoreType scoreType, int patchSize, int fastThreshold)
{
    //CV_Assert(firstLevel >= 0);
    return makePtr<ORB_Impl__>(nfeatures, scaleFactor, nlevels, edgeThreshold,
        firstLevel, wta_k, scoreType, patchSize, fastThreshold);
}






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

	Ptr<Feature2D__> det;
	Mat des; 
	//Ptr<SURF> det_surf;
	
	/*if (detector == Sift)
	{
		det = SIFT__::create();
		det->Feature2D::detect(image, *kp, mask);
		det->Feature2D::compute(image, *kp, des);

	}
	else */if (detector == Orb)
	{
		det = ORB__::create();
		det->Feature2D__::detect(image, *kp, mask);
		det->Feature2D__::compute(image, *kp, des);
	}


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
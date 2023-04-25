
#include <stdlib.h>
#include <iostream>

typedef unsigned char uchar;

#define small_width 414
#define small_height 376

#define whole_width 1241
#define whole_height 376

#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_SUBMAT_FLAG_SHIFT    15
#define CV_SUBMAT_FLAG          (1 << CV_SUBMAT_FLAG_SHIFT)
#define CV_CN_SHIFT   3
#define CV_CN_MAX     512
#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)

class Mat
{
//private:
    //static unsigned char ch3[4*376*1241];
    
public:

    void create_disp(unsigned char *address);

    void create_ones(unsigned char *address);

    Mat();

    //Mat(int rows, int cols, int type);

    //Mat(Size size, int type);

    //Mat(int rows, int cols, int type, const Scalar& s);

    //Mat(Size size, int type, const Scalar& s);

    //Mat(int ndims, const int* sizes, int type);

    //Mat(const std::vector<int>& sizes, int type);

    //Mat(int ndims, const int* sizes, int type, const Scalar& s);

    //Mat(const std::vector<int>& sizes, int type, const Scalar& s);

    //Mat(const Mat& m);

    //Mat(int rows, int cols, int type, void* data, size_t step = AUTO_STEP);

    //Mat(Size size, int type, void* data, size_t step = AUTO_STEP);

    //Mat(int ndims, const int* sizes, int type, void* data, const size_t* steps = 0);

    //Mat(const std::vector<int>& sizes, int type, void* data, const size_t* steps = 0);

    //Mat(const Mat& m, const Range& rowRange, const Range& colRange = Range::all());

    //Mat(const Mat& m, const Rect& roi);

    //Mat(const Mat& m, const Range* ranges);

    //Mat(const Mat& m, const std::vector<Range>& ranges);

    //template<typename _Tp> explicit Mat(const std::vector<_Tp>& vec, bool copyData = false);

    //template<typename _Tp, typename = typename std::enable_if<std::is_arithmetic<_Tp>::value>::type>
    //explicit Mat(const std::initializer_list<_Tp> list);

    //template<typename _Tp> explicit Mat(const std::initializer_list<int> sizes, const std::initializer_list<_Tp> list);

    //template<typename _Tp, size_t _Nm> explicit Mat(const std::array<_Tp, _Nm>& arr, bool copyData = false);

    //template<typename _Tp, int n> explicit Mat(const Vec<_Tp, n>& vec, bool copyData = true);

    //template<typename _Tp, int m, int n> explicit Mat(const Matx<_Tp, m, n>& mtx, bool copyData = true);

    //template<typename _Tp> explicit Mat(const Point_<_Tp>& pt, bool copyData = true);

    //template<typename _Tp> explicit Mat(const Point3_<_Tp>& pt, bool copyData = true);

    //template<typename _Tp> explicit Mat(const MatCommaInitializer_<_Tp>& commaInitializer);

    //explicit Mat(const cuda::GpuMat& m);

    //~Mat();

    //Mat& operator = (const Mat& m);

    //Mat& operator = (const MatExpr& expr);

    //UMat getUMat(AccessFlag accessFlags, UMatUsageFlags usageFlags = USAGE_DEFAULT) const;

    //Mat row(int y) const;

    //Mat col(int x) const;

    //Mat rowRange(int startrow, int endrow) const;

    //Mat rowRange(const Range& r) const;

    //Mat colRange(int startcol, int endcol) const;

    //Mat colRange(const Range& r) const;

    //Mat diag(int d = 0) const;

    //static Mat diag(const Mat& d);

    //Mat clone() const CV_NODISCARD;

    //void copyTo(OutputArray m) const;

    //void copyTo(OutputArray m, InputArray mask) const;

    //void convertTo(Mat& m, int rtype, double alpha = 1, double beta = 0) const;

    //void assignTo(Mat& m, int type = -1) const;

    //Mat& operator = (const Scalar& s);

    //Mat& setTo(InputArray value, InputArray mask = noArray());

    //Mat reshape(int cn, int rows = 0) const;

    /** @overload */
    //Mat reshape(int cn, int newndims, const int* newsz) const;

    /** @overload */
    //Mat reshape(int cn, const std::vector<int>& newshape) const;

    //MatExpr t() const;

    //MatExpr inv(int method = DECOMP_LU) const;

    //MatExpr mul(InputArray m, double scale = 1) const;

    //Mat cross(InputArray m) const;

    //double dot(InputArray m) const;

    //static MatExpr zeros(int rows, int cols, int type);

    //static MatExpr zeros(Size size, int type);

    //static MatExpr zeros(int ndims, const int* sz, int type);

    //static MatExpr ones(int rows, int cols, int type);

    //static MatExpr ones(Size size, int type);

    //static MatExpr ones(int ndims, const int* sz, int type);

    //static MatExpr eye(int rows, int cols, int type);

    //static MatExpr eye(Size size, int type);

    //void create(int rows, int cols, int type);

    //void create(Size size, int type);

    //void create(int ndims, const int* sizes, int type);

    //void create(const std::vector<int>& sizes, int type);

    //void addref();

//  void release();

    //! internal use function, consider to use 'release' method instead; deallocates the matrix data
    //void deallocate();
    //! internal use function; properly re-allocates _size, _step arrays
    //void copySize(const Mat& m);

    //void reserve(size_t sz);

    //void reserveBuffer(size_t sz);

    //void resize(size_t sz);

    //void resize(size_t sz, const Scalar& s);

    //! internal function
    //void push_back_(const void* elem);

    //template<typename _Tp> void push_back(const _Tp& elem);

    //template<typename _Tp> void push_back(const Mat_<_Tp>& elem);

    //template<typename _Tp> void push_back(const std::vector<_Tp>& elem);

    //void push_back(const Mat& m);

    //void pop_back(size_t nelems = 1);

    //void locateROI(Size& wholeSize, Point& ofs) const;

    //Mat& adjustROI(int dtop, int dbottom, int dleft, int dright);

    //Mat operator()(Range rowRange, Range colRange) const;

    //Mat operator()(const Rect& roi) const;

    //Mat operator()(const Range* ranges) const;

    //Mat operator()(const std::vector<Range>& ranges) const;

    //template<typename _Tp> operator std::vector<_Tp>() const;
    //template<typename _Tp, int n> operator Vec<_Tp, n>() const;
    //template<typename _Tp, int m, int n> operator Matx<_Tp, m, n>() const;

    //template<typename _Tp, std::size_t _Nm> operator std::array<_Tp, _Nm>() const;

    //bool isContinuous() const;

    //! returns true if the matrix is a submatrix of another matrix
    //bool isSubmatrix() const;

    //size_t elemSize() const;

    //size_t elemSize1() const;

    /** @brief Returns the type of a matrix element.

    The method returns a matrix element type. This is an identifier compatible with the CvMat type
    system, like CV_16SC3 or 16-bit signed 3-channel array, and so on.
     */
    //int type() const;

    //int depth() const;

    int channels() const;

    //size_t step1(int i = 0) const;

    //bool empty() const;

    //size_t total() const;

    //size_t total(int startDim, int endDim = INT_MAX) const;

    //int checkVector(int elemChannels, int depth = -1, bool requireContinuous = true) const;

    //uchar* ptr(int i0 = 0);
    /** @overload */
    //const uchar* ptr(int i0 = 0) const;

    //uchar* ptr(int row, int col);
   
    //const uchar* ptr(int row, int col) const;

    /** @overload */
    //uchar* ptr(int i0, int i1, int i2);
    /** @overload */
    //const uchar* ptr(int i0, int i1, int i2) const;

    /** @overload */
    //uchar* ptr(const int* idx);
    /** @overload */
    //const uchar* ptr(const int* idx) const;
    /** @overload */
    //template<int n> uchar* ptr(const Vec<int, n>& idx);
    /** @overload */
    //template<int n> const uchar* ptr(const Vec<int, n>& idx) const;

    /** @overload */
    template<typename _Tp> _Tp* ptr(int i0 = 0);
    /** @overload */
    template<typename _Tp> const _Tp* ptr(int i0 = 0) const;
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
    //template<typename _Tp> _Tp* ptr(int row, int col);
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
    //template<typename _Tp> const _Tp* ptr(int row, int col) const;
    /** @overload */
    //template<typename _Tp> _Tp* ptr(int i0, int i1, int i2);
    /** @overload */
    //template<typename _Tp> const _Tp* ptr(int i0, int i1, int i2) const;
    /** @overload */
    //template<typename _Tp> _Tp* ptr(const int* idx);
    /** @overload */
    //template<typename _Tp> const _Tp* ptr(const int* idx) const;
    /** @overload */
    //template<typename _Tp, int n> _Tp* ptr(const Vec<int, n>& idx);
    /** @overload */
    //template<typename _Tp, int n> const _Tp* ptr(const Vec<int, n>& idx) const;

    //template<typename _Tp> _Tp& at(int i0 = 0);
    /** @overload
    @param i0 Index along the dimension 0
    */
    //template<typename _Tp> const _Tp& at(int i0 = 0) const;
    template<typename _Tp> _Tp& at(int row, int col);

    //template<typename _Tp> const _Tp& at(int row, int col) const;

  
    //template<typename _Tp> _Tp& at(int i0, int i1, int i2);

    //template<typename _Tp> const _Tp& at(int i0, int i1, int i2) const;

    //template<typename _Tp> _Tp& at(const int* idx);

    //template<typename _Tp> const _Tp& at(const int* idx) const;

    /** @overload */
    //template<typename _Tp, int n> _Tp& at(const Vec<int, n>& idx);
    /** @overload */
    //template<typename _Tp, int n> const _Tp& at(const Vec<int, n>& idx) const;

    //template<typename _Tp> _Tp& at(Point pt);

    //template<typename _Tp> const _Tp& at(Point pt) const;

    //template<typename _Tp> MatIterator_<_Tp> begin();
    //template<typename _Tp> MatConstIterator_<_Tp> begin() const;

    //template<typename _Tp> MatIterator_<_Tp> end();
    //template<typename _Tp> MatConstIterator_<_Tp> end() const;

    //template<typename _Tp, typename Functor> void forEach(const Functor& operation);
    /** @overload */
    //template<typename _Tp, typename Functor> void forEach(const Functor& operation) const;

    //Mat(Mat&& m);
    //Mat& operator = (Mat&& m);

    enum { MAGIC_VAL = 0x42FF0000, AUTO_STEP = 0, CONTINUOUS_FLAG = CV_MAT_CONT_FLAG, SUBMATRIX_FLAG = CV_SUBMAT_FLAG };
    enum { MAGIC_MASK = 0xFFFF0000, TYPE_MASK = 0x00000FFF, DEPTH_MASK = 7 };

    int flags;
    //! the matrix dimensionality, >= 2
    int dims;
    //! the number of rows and columns or (-1, -1) when the matrix has more than 2 dimensions
    int rows, cols;
    //! pointer to the data
    uchar* data;

    //! helper fields used in locateROI and adjustROI
    //const uchar* datastart;
    //const uchar* dataend;
    //const uchar* datalimit;

    //! custom allocator
    //MatAllocator* allocator;
    //! and the standard allocator
    //static MatAllocator* getStdAllocator();
    //static MatAllocator* getDefaultAllocator();
    //static void setDefaultAllocator(MatAllocator* allocator);

    //! internal use method: updates the continuity flag
    //void updateContinuityFlag();

    //! interaction with UMat
    //UMatData* u;

    //MatSize size;
    //MatStep step;
    size_t step[2];
    int size[2];

protected:
    //template<typename _Tp, typename Functor> void forEach_impl(const Functor& operation);
};

//---------------------------------------------------------------------------------------------

Mat::Mat()
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(0)//, step(0)
{
    step[0] = 0;
    step[1] = 0;
    size[0] = rows;
    size[1] = cols;
}

inline
void Mat::create_disp(unsigned char *address)
{
    size[0] = small_height;
    size[1] = small_width;
    flags = 1124024323;
    dims = 2;
    step[0] = small_width * 2;
    step[1] = 2;
    rows = small_height;
    cols = small_width;
    //static unsigned char ch2[376*1241*2];

    data = address;

    /*
    int imcol2 = cols*2;
    for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols*2; k++)
        {
            data[k + (j * imcol2)] = 205;
        }
    }*/

    for (int i = 0; i < small_height * small_width * 2; i++)
    {
        data[i] = 205;
    }
    /*
    for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            at<short>(j,k) = -12851;
        }
    }*/
}

template<typename _Tp> inline
_Tp& Mat::at(int i0, int i1)
{
    //CV_DbgAssert(dims <= 2);
    //CV_DbgAssert(data);
    //CV_DbgAssert((unsigned)i0 < (unsigned)size.p[0]);
    //CV_DbgAssert((unsigned)(i1 * DataType<_Tp>::channels) < (unsigned)(size.p[1] * channels()));
    //CV_DbgAssert(CV_ELEM_SIZE1(traits::Depth<_Tp>::value) == elemSize1());
    return ((_Tp*)(data + step[0] * i0))[i1];
}



template<typename _Tp> inline
_Tp* Mat::ptr(int y)
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (_Tp*)(data + step[0] * y);
}

inline
int Mat::channels() const
{
    return CV_MAT_CN(flags);
}

template<typename _Tp> inline
const _Tp* Mat::ptr(int y) const
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (const _Tp*)(data + step[0] * y);
}

//--------------------------------------------------------------------------------------------------

inline
void Mat::create_ones(unsigned char *address)
{
    size[0] = small_height;
    size[1] = small_width;
    flags = 1124024325;
    dims = 2;
    step[0] = small_width * 4;
    step[1] = 4;
    rows = small_height;
    cols = small_width;
    //static unsigned char ch3[4*376*1241];

    data = address;

    int imcol3 = cols;
    for (int i = 0; i < small_height * small_width; i++)
    {
        data[i * 4] = 0;
        data[i * 4 + 1] = 0;
        data[i * 4 + 2] = 128;
        data[i * 4 + 3] = 63;
    }
}

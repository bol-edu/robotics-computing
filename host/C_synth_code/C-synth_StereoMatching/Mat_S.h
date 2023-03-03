

#include <stdlib.h>
#include <iostream>

typedef unsigned char uchar;

#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_SUBMAT_FLAG_SHIFT    15
#define CV_SUBMAT_FLAG          (1 << CV_SUBMAT_FLAG_SHIFT)
#define CV_CN_SHIFT   3
#define CV_CN_MAX     512
#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)

class Mat_S
{
public:

    void create_disp();

    void create_ones();

    Mat_S();

    //Mat_S(int rows, int cols, int type);

    //Mat_S(Size size, int type);

    //Mat_S(int rows, int cols, int type, const Scalar& s);

    //Mat_S(Size size, int type, const Scalar& s);

    //Mat_S(int ndims, const int* sizes, int type);

    //Mat_S(const std::vector<int>& sizes, int type);

    //Mat_S(int ndims, const int* sizes, int type, const Scalar& s);

    //Mat_S(const std::vector<int>& sizes, int type, const Scalar& s);

    //Mat_S(const Mat_S& m);

    //Mat_S(int rows, int cols, int type, void* data, size_t step = AUTO_STEP);

    //Mat_S(Size size, int type, void* data, size_t step = AUTO_STEP);

    //Mat_S(int ndims, const int* sizes, int type, void* data, const size_t* steps = 0);

    //Mat_S(const std::vector<int>& sizes, int type, void* data, const size_t* steps = 0);

    //Mat_S(const Mat_S& m, const Range& rowRange, const Range& colRange = Range::all());

    //Mat_S(const Mat_S& m, const Rect& roi);

    //Mat_S(const Mat_S& m, const Range* ranges);

    //Mat_S(const Mat_S& m, const std::vector<Range>& ranges);

    //template<typename _Tp> explicit Mat_S(const std::vector<_Tp>& vec, bool copyData = false);

    //template<typename _Tp, typename = typename std::enable_if<std::is_arithmetic<_Tp>::value>::type>
    //explicit Mat_S(const std::initializer_list<_Tp> list);

    //template<typename _Tp> explicit Mat_S(const std::initializer_list<int> sizes, const std::initializer_list<_Tp> list);

    //template<typename _Tp, size_t _Nm> explicit Mat_S(const std::array<_Tp, _Nm>& arr, bool copyData = false);

    //template<typename _Tp, int n> explicit Mat_S(const Vec<_Tp, n>& vec, bool copyData = true);

    //template<typename _Tp, int m, int n> explicit Mat_S(const Matx<_Tp, m, n>& mtx, bool copyData = true);

    //template<typename _Tp> explicit Mat_S(const Point_<_Tp>& pt, bool copyData = true);

    //template<typename _Tp> explicit Mat_S(const Point3_<_Tp>& pt, bool copyData = true);

    //template<typename _Tp> explicit Mat_S(const MatCommaInitializer_<_Tp>& commaInitializer);

    //explicit Mat_S(const cuda::GpuMat& m);

    //~Mat_S();

    //Mat_S& operator = (const Mat_S& m);

    //Mat_S& operator = (const MatExpr& expr);

    //UMat getUMat(AccessFlag accessFlags, UMatUsageFlags usageFlags = USAGE_DEFAULT) const;

    //Mat_S row(int y) const;

    //Mat_S col(int x) const;

    //Mat_S rowRange(int startrow, int endrow) const;

    //Mat_S rowRange(const Range& r) const;

    //Mat_S colRange(int startcol, int endcol) const;

    //Mat_S colRange(const Range& r) const;

    //Mat_S diag(int d = 0) const;

    //static Mat_S diag(const Mat_S& d);

    //Mat_S clone() const CV_NODISCARD;

    //void copyTo(OutputArray m) const;

    //void copyTo(OutputArray m, InputArray mask) const;

    //void convertTo(Mat_S& m, int rtype, double alpha = 1, double beta = 0) const;

    //void assignTo(Mat_S& m, int type = -1) const;

    //Mat_S& operator = (const Scalar& s);

    //Mat_S& setTo(InputArray value, InputArray mask = noArray());

    //Mat_S reshape(int cn, int rows = 0) const;

    /** @overload */
    //Mat_S reshape(int cn, int newndims, const int* newsz) const;

    /** @overload */
    //Mat_S reshape(int cn, const std::vector<int>& newshape) const;

    //MatExpr t() const;

    //MatExpr inv(int method = DECOMP_LU) const;

    //MatExpr mul(InputArray m, double scale = 1) const;

    //Mat_S cross(InputArray m) const;

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
    //void copySize(const Mat_S& m);

    //void reserve(size_t sz);

    //void reserveBuffer(size_t sz);

    //void resize(size_t sz);

    //void resize(size_t sz, const Scalar& s);

    //! internal function
    //void push_back_(const void* elem);

    //template<typename _Tp> void push_back(const _Tp& elem);

    //template<typename _Tp> void push_back(const Mat_<_Tp>& elem);

    //template<typename _Tp> void push_back(const std::vector<_Tp>& elem);

    //void push_back(const Mat_S& m);

    //void pop_back(size_t nelems = 1);

    //void locateROI(Size& wholeSize, Point& ofs) const;

    //Mat_S& adjustROI(int dtop, int dbottom, int dleft, int dright);

    //Mat_S operator()(Range rowRange, Range colRange) const;

    //Mat_S operator()(const Rect& roi) const;

    //Mat_S operator()(const Range* ranges) const;

    //Mat_S operator()(const std::vector<Range>& ranges) const;

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

    //Mat_S(Mat_S&& m);
    //Mat_S& operator = (Mat_S&& m);

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
    int* size;

protected:
    //template<typename _Tp, typename Functor> void forEach_impl(const Functor& operation);
};

//---------------------------------------------------------------------------------------------

Mat_S::Mat_S()
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(0), size(&rows)//, step(0)
{
    step[0] = 0;
    step[1] = 0;
}

inline
void Mat_S::create_disp()
{
    size[0] = 376;
    size[1] = 1241;
    flags = 1124024323;
    dims = 2;
    step[0] = 2482;
    step[1] = 2;
    rows = 376;
    cols = 1241;
    static unsigned char ch2[376*1241*2];

    data = ch2;

    /*
    int imcol2 = cols*2;
    for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols*2; k++)
        {
            data[k + (j * imcol2)] = 205;
        }
    }*/

    for (int i = 0; i < 376 * 1241 * 2; i++)
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
_Tp& Mat_S::at(int i0, int i1)
{
    //CV_DbgAssert(dims <= 2);
    //CV_DbgAssert(data);
    //CV_DbgAssert((unsigned)i0 < (unsigned)size.p[0]);
    //CV_DbgAssert((unsigned)(i1 * DataType<_Tp>::channels) < (unsigned)(size.p[1] * channels()));
    //CV_DbgAssert(CV_ELEM_SIZE1(traits::Depth<_Tp>::value) == elemSize1());
    return ((_Tp*)(data + step[0] * i0))[i1];
}



template<typename _Tp> inline
_Tp* Mat_S::ptr(int y)
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (_Tp*)(data + step[0] * y);
}

inline
int Mat_S::channels() const
{
    return CV_MAT_CN(flags);
}

template<typename _Tp> inline
const _Tp* Mat_S::ptr(int y) const
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (const _Tp*)(data + step[0] * y);
}

//--------------------------------------------------------------------------------------------------

inline
void Mat_S::create_ones()
{
    size[0] = 376;
    size[1] = 1241;
    flags = 1124024325;
    dims = 2;
    step[0] = 4964;
    step[1] = 4;
    rows = 376;
    cols = 1241;
    static unsigned char ch3[4*376*1241];

    data = ch3;

    int imcol3 = cols;
    for (int i = 0; i < 376*1241; i++)
    {
        data[i * 4] = 0;
        data[i * 4 + 1] = 0;
        data[i * 4 + 2] = 128;
        data[i * 4 + 3] = 63;
    }
}


#include <stdlib.h>
#include <iostream>
#include "Rect.h"
#include "Range.h"
#include "Point__.h"
#include "define.h"
//#include "AutoBuffer__.h"
#include "Size.h"


class Mat
{
public:
    
    Mat();

    
    //Mat(int rows, int cols, int type);

    
    //Mat(Size size, int type);

    
    //Mat(int rows, int cols, int type, const Scalar& s);

    
    //Mat(Size size, int type, const Scalar& s);

    
   // Mat(int ndims, const int* sizes, int type);

    
   // Mat(const std::vector<int>& sizes, int type);

    
    //Mat(int ndims, const int* sizes, int type, const Scalar& s);

    
   // Mat(const std::vector<int>& sizes, int type, const Scalar& s);


    
    //Mat(const Mat& m);

    
    //Mat(int rows, int cols, int type, void* data, size_t step = AUTO_STEP);

   
    Mat(Size size, int type, void* data, size_t step = AUTO_STEP);

   
    //Mat(int ndims, const int* sizes, int type, void* data, const size_t* steps = 0);

    
   // Mat(const std::vector<int>& sizes, int type, void* data, const size_t* steps = 0);

    
   // Mat(const Mat& m, const Range& rowRange, const Range& colRange = Range::all());

    
    Mat(const Mat m, const Rect& roi);

    
   // Mat(const Mat& m, const Range* ranges);

    
    //Mat(const Mat& m, const std::vector<Range>& ranges);

    
   // template<typename _Tp> explicit Mat(const std::vector<_Tp>& vec, bool copyData = false);

    /** @overload
    */
    //template<typename _Tp, typename = typename std::enable_if<std::is_arithmetic<_Tp>::value>::type>
    //explicit Mat(const std::initializer_list<_Tp> list);

    /** @overload
    */
    //template<typename _Tp> explicit Mat(const std::initializer_list<int> sizes, const std::initializer_list<_Tp> list);

    /** @overload
    */
    //template<typename _Tp, size_t _Nm> explicit Mat(const std::array<_Tp, _Nm>& arr, bool copyData = false);

    /** @overload
    */
    //template<typename _Tp, int n> explicit Mat(const Vec<_Tp, n>& vec, bool copyData = true);

    /** @overload
    */
    //template<typename _Tp, int m, int n> explicit Mat(const Matx<_Tp, m, n>& mtx, bool copyData = true);

    /** @overload
    */
    //template<typename _Tp> explicit Mat(const Point_<_Tp>& pt, bool copyData = true);

    /** @overload
    */
    //template<typename _Tp> explicit Mat(const Point3_<_Tp>& pt, bool copyData = true);

    /** @overload
    */
    //template<typename _Tp> explicit Mat(const MatCommaInitializer_<_Tp>& commaInitializer);

    //! download data from GpuMat
    //explicit Mat(const cuda::GpuMat& m);

    //! destructor - calls release()
    ~Mat();

    
    Mat operator = (const Mat m);

    
    //Mat& operator = (const MatExpr& expr);

    

   
    //Mat row(int y) const;

    
   // Mat col(int x) const;

    
    //Mat rowRange(int startrow, int endrow) const;

    
   // Mat rowRange(const Range& r) const;

   
   // Mat colRange(int startcol, int endcol) const;

    /** @overload
    @param r Range structure containing both the start and the end indices.
    */
   // Mat colRange(const Range& r) const;

    
   // Mat diag(int d = 0) const;

    
    //static Mat diag(const Mat& d);

    
   // Mat clone() const CV_NODISCARD;

   
   // void copyTo(OutputArray m) const;

    
   // void copyTo(OutputArray m, InputArray mask) const;

    
   // void convertTo(OutputArray m, int rtype, double alpha = 1, double beta = 0) const;

    
  //  void assignTo(Mat& m, int type = -1) const;

    /** @brief Sets all or some of the array elements to the specified value.
    @param s Assigned scalar converted to the actual array type.
    */
  //  Mat& operator = (const Scalar& s);

   
   // Mat& setTo(InputArray value, InputArray mask = noArray());

    
   // Mat reshape(int cn, int rows = 0) const;

    /** @overload */
 //   Mat reshape(int cn, int newndims, const int* newsz) const;

    /** @overload */
 //   Mat reshape(int cn, const std::vector<int>& newshape) const;

   
 //   MatExpr t() const;

    
 //   MatExpr inv(int method = DECOMP_LU) const;

    
//    MatExpr mul(InputArray m, double scale = 1) const;

   
 //   Mat cross(InputArray m) const;

   
 //   double dot(InputArray m) const;

    
//    static MatExpr zeros(int rows, int cols, int type);

    /** @overload
    @param size Alternative to the matrix size specification Size(cols, rows) .
    @param type Created matrix type.
    */
//    static MatExpr zeros(Size size, int type);

    /** @overload
    @param ndims Array dimensionality.
    @param sz Array of integers specifying the array shape.
    @param type Created matrix type.
    */
//    static MatExpr zeros(int ndims, const int* sz, int type);

   
//    static MatExpr ones(int rows, int cols, int type);

    /** @overload
    @param size Alternative to the matrix size specification Size(cols, rows) .
    @param type Created matrix type.
    */
//    static MatExpr ones(Size size, int type);

    /** @overload
    @param ndims Array dimensionality.
    @param sz Array of integers specifying the array shape.
    @param type Created matrix type.
    */
//    static MatExpr ones(int ndims, const int* sz, int type);

    
//    static MatExpr eye(int rows, int cols, int type);

    /** @overload
    @param size Alternative matrix size specification as Size(cols, rows) .
    @param type Created matrix type.
    */
//    static MatExpr eye(Size size, int type);

    void exRect(const Mat m, const Rect roi);
    
    void create();
    void createP();
    void create1();
    void create_des();
    void create_ker();
    //void create(int rows, int cols, int type);

    /** @overload
    @param size Alternative new matrix size specification: Size(cols, rows)
    @param type New matrix type.
    */
    //void create(Size size, int type);

    /** @overload
    @param ndims New array dimensionality.
    @param sizes Array of integers specifying a new array shape.
    @param type New matrix type.
    */
    //void create(int ndims, const int* sizes, int type);

    
    //void create(const std::vector<int>& sizes, int type);

    
    //void addref();

    
    void release();

    //! internal use function, consider to use 'release' method instead; deallocates the matrix data
 //   void deallocate();
    //! internal use function; properly re-allocates _size, _step arrays
    void copySize(const Mat& m);

   
    //void reserve(size_t sz);

    
//    void reserveBuffer(size_t sz);

   
    //void resize(size_t sz);

    /** @overload
    @param sz New number of rows.
    @param s Value assigned to the newly added elements.
     */
//    void resize(size_t sz, const Scalar& s);

    //! internal function
//    void push_back_(const void* elem);

    
//    template<typename _Tp> void push_back(const _Tp& elem);

    /** @overload
    @param elem Added element(s).
    */
//    template<typename _Tp> void push_back(const Mat_<_Tp>& elem);

    /** @overload
    @param elem Added element(s).
    */
 //   template<typename _Tp> void push_back(const std::vector<_Tp>& elem);

    /** @overload
    @param m Added line(s).
    */
//    void push_back(const Mat& m);

    /** @brief Removes elements from the bottom of the matrix.

    The method removes one or more rows from the bottom of the matrix.
    @param nelems Number of removed rows. If it is greater than the total number of rows, an exception
    is thrown.
     */
 //   void pop_back(size_t nelems = 1);

    /** @brief Locates the matrix header within a parent matrix.

    After you extracted a submatrix from a matrix using Mat::row, Mat::col, Mat::rowRange,
    Mat::colRange, and others, the resultant submatrix points just to the part of the original big
    matrix. However, each submatrix contains information (represented by datastart and dataend
    fields) that helps reconstruct the original matrix size and the position of the extracted
    submatrix within the original matrix. The method locateROI does exactly that.
    @param wholeSize Output parameter that contains the size of the whole matrix containing *this*
    as a part.
    @param ofs Output parameter that contains an offset of *this* inside the whole matrix.
     */
    void locateROI(Size& wholeSize, Point__& ofs) const;

    /** @brief Adjusts a submatrix size and position within the parent matrix.

    The method is complimentary to Mat::locateROI . The typical use of these functions is to determine
    the submatrix position within the parent matrix and then shift the position somehow. Typically, it
    can be required for filtering operations when pixels outside of the ROI should be taken into
    account. When all the method parameters are positive, the ROI needs to grow in all directions by the
    specified amount, for example:
    @code
        A.adjustROI(2, 2, 2, 2);
    @endcode
    In this example, the matrix size is increased by 4 elements in each direction. The matrix is shifted
    by 2 elements to the left and 2 elements up, which brings in all the necessary pixels for the
    filtering with the 5x5 kernel.

    adjustROI forces the adjusted ROI to be inside of the parent matrix that is boundaries of the
    adjusted ROI are constrained by boundaries of the parent matrix. For example, if the submatrix A is
    located in the first row of a parent matrix and you called A.adjustROI(2, 2, 2, 2) then A will not
    be increased in the upward direction.

    The function is used internally by the OpenCV filtering functions, like filter2D , morphological
    operations, and so on.
    @param dtop Shift of the top submatrix boundary upwards.
    @param dbottom Shift of the bottom submatrix boundary downwards.
    @param dleft Shift of the left submatrix boundary to the left.
    @param dright Shift of the right submatrix boundary to the right.
    @sa copyMakeBorder
     */
    //Mat& adjustROI(int dtop, int dbottom, int dleft, int dright);

    /** @brief Extracts a rectangular submatrix.

    The operators make a new header for the specified sub-array of \*this . They are the most
    generalized forms of Mat::row, Mat::col, Mat::rowRange, and Mat::colRange . For example,
    `A(Range(0, 10), Range::all())` is equivalent to `A.rowRange(0, 10)`. Similarly to all of the above,
    the operators are O(1) operations, that is, no matrix data is copied.
    @param rowRange Start and end row of the extracted submatrix. The upper boundary is not included. To
    select all the rows, use Range::all().
    @param colRange Start and end column of the extracted submatrix. The upper boundary is not included.
    To select all the columns, use Range::all().
     */
 //   Mat operator()(Range rowRange, Range colRange) const;

    /** @overload
    @param roi Extracted submatrix specified as a rectangle.
    */
    Mat operator()(const Rect& roi) const;

    /** @overload
    @param ranges Array of selected ranges along each array dimension.
    */
    //Mat operator()(const Range* ranges) const;

    /** @overload
    @param ranges Array of selected ranges along each array dimension.
    */
 //   Mat operator()(const std::vector<Range>& ranges) const;

 //   template<typename _Tp> operator std::vector<_Tp>() const;
 //   template<typename _Tp, int n> operator Vec<_Tp, n>() const;
  //  template<typename _Tp, int m, int n> operator Matx<_Tp, m, n>() const;

 //   template<typename _Tp, std::size_t _Nm> operator std::array<_Tp, _Nm>() const;

   
    bool isContinuous() const;

    //! returns true if the matrix is a submatrix of another matrix
    bool isSubmatrix() const;

    /** @brief Returns the matrix element size in bytes.

    The method returns the matrix element size in bytes. For example, if the matrix type is CV_16SC3 ,
    the method returns 3\*sizeof(short) or 6.
     */
    size_t elemSize() const;

    /** @brief Returns the size of each matrix element channel in bytes.

    The method returns the matrix element channel size in bytes, that is, it ignores the number of
    channels. For example, if the matrix type is CV_16SC3 , the method returns sizeof(short) or 2.
     */
    size_t elemSize1() const;

    /** @brief Returns the type of a matrix element.

    The method returns a matrix element type. This is an identifier compatible with the CvMat type
    system, like CV_16SC3 or 16-bit signed 3-channel array, and so on.
     */
    int type() const;

    /** @brief Returns the depth of a matrix element.

    The method returns the identifier of the matrix element depth (the type of each individual channel).
    For example, for a 16-bit signed element array, the method returns CV_16S . A complete list of
    matrix types contains the following values:
    -   CV_8U - 8-bit unsigned integers ( 0..255 )
    -   CV_8S - 8-bit signed integers ( -128..127 )
    -   CV_16U - 16-bit unsigned integers ( 0..65535 )
    -   CV_16S - 16-bit signed integers ( -32768..32767 )
    -   CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
    -   CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )
    -   CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
     */
    int depth() const;

    
    int channels() const;

    /** @brief Returns a normalized step.

    The method returns a matrix step divided by Mat::elemSize1() . It can be useful to quickly access an
    arbitrary matrix element.
     */
    size_t step1(int i = 0) const;

    /** @brief Returns true if the array has no elements.

    The method returns true if Mat::total() is 0 or if Mat::data is NULL. Because of pop_back() and
    resize() methods `M.total() == 0` does not imply that `M.data == NULL`.
     */
    bool empty() const;

    /** @brief Returns the total number of array elements.

    The method returns the number of array elements (a number of pixels if the array represents an
    image).
     */
    size_t total() const;

    /** @brief Returns the total number of array elements.

     The method returns the number of elements within a certain sub-array slice with startDim <= dim < endDim
     */
 //   size_t total(int startDim, int endDim = INT_MAX) const;

    
 //   int checkVector(int elemChannels, int depth = -1, bool requireContinuous = true) const;

    /** @brief Returns a pointer to the specified matrix row.

    The methods return `uchar*` or typed pointer to the specified matrix row. See the sample in
    Mat::isContinuous to know how to use these methods.
    @param i0 A 0-based row index.
     */
    uchar* ptr(int i0 = 0);
    /** @overload */
    const uchar* ptr(int i0 = 0) const;

    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
  //  uchar* ptr(int row, int col);
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
 //   const uchar* ptr(int row, int col) const;

    /** @overload */
 //   uchar* ptr(int i0, int i1, int i2);
    /** @overload */
 //   const uchar* ptr(int i0, int i1, int i2) const;

    /** @overload */
    //uchar* ptr(const int* idx);
    /** @overload */
    //const uchar* ptr(const int* idx) const;
    /** @overload */
 //   template<int n> uchar* ptr(const Vec<int, n>& idx);
    /** @overload */
 //   template<int n> const uchar* ptr(const Vec<int, n>& idx) const;

    /** @overload */
    template<typename _Tp> _Tp* ptr(int i0 = 0);
    /** @overload */
    template<typename _Tp> const _Tp* ptr(int i0 = 0) const;
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
 //   template<typename _Tp> _Tp* ptr(int row, int col);
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
  //  template<typename _Tp> const _Tp* ptr(int row, int col) const;
    /** @overload */
 //   template<typename _Tp> _Tp* ptr(int i0, int i1, int i2);
    /** @overload */
 //   template<typename _Tp> const _Tp* ptr(int i0, int i1, int i2) const;
    /** @overload */
 //   template<typename _Tp> _Tp* ptr(const int* idx);
    /** @overload */
 //   template<typename _Tp> const _Tp* ptr(const int* idx) const;
    /** @overload */
 //   template<typename _Tp, int n> _Tp* ptr(const Vec<int, n>& idx);
    /** @overload */
 //   template<typename _Tp, int n> const _Tp* ptr(const Vec<int, n>& idx) const;

    
    template<typename _Tp> _Tp& at(int i0 = 0);
    /** @overload
    @param i0 Index along the dimension 0
    */
 //   template<typename _Tp> const _Tp& at(int i0 = 0) const;
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
    template<typename _Tp> _Tp& at(int row, int col);
    /** @overload
    @param row Index along the dimension 0
    @param col Index along the dimension 1
    */
    template<typename _Tp> const _Tp& at(int row, int col) const;

    /** @overload
    @param i0 Index along the dimension 0
    @param i1 Index along the dimension 1
    @param i2 Index along the dimension 2
    */
  //  template<typename _Tp> _Tp& at(int i0, int i1, int i2);
    /** @overload
    @param i0 Index along the dimension 0
    @param i1 Index along the dimension 1
    @param i2 Index along the dimension 2
    */
  //  template<typename _Tp> const _Tp& at(int i0, int i1, int i2) const;

    /** @overload
    @param idx Array of Mat::dims indices.
    */
  //  template<typename _Tp> _Tp& at(const int* idx);
    /** @overload
    @param idx Array of Mat::dims indices.
    */
 //   template<typename _Tp> const _Tp& at(const int* idx) const;

    /** @overload */
  //  template<typename _Tp, int n> _Tp& at(const Vec<int, n>& idx);
    /** @overload */
 //   template<typename _Tp, int n> const _Tp& at(const Vec<int, n>& idx) const;

    /** @overload
    special versions for 2D arrays (especially convenient for referencing image pixels)
    @param pt Element position specified as Point(j,i) .
    */
  //  template<typename _Tp> _Tp& at(Point pt);
    /** @overload
    special versions for 2D arrays (especially convenient for referencing image pixels)
    @param pt Element position specified as Point(j,i) .
    */
 //   template<typename _Tp> const _Tp& at(Point pt) const;

   
 //   template<typename _Tp> MatIterator_<_Tp> begin();
 //   template<typename _Tp> MatConstIterator_<_Tp> begin() const;

    /** @brief Returns the matrix iterator and sets it to the after-last matrix element.

    The methods return the matrix read-only or read-write iterators, set to the point following the last
    matrix element.
     */
 //   template<typename _Tp> MatIterator_<_Tp> end();
 //   template<typename _Tp> MatConstIterator_<_Tp> end() const;

   
  //  template<typename _Tp, typename Functor> void forEach(const Functor& operation);
    /** @overload */
 //   template<typename _Tp, typename Functor> void forEach(const Functor& operation) const;

    //Mat(Mat&& m);
    //Mat& operator = (Mat&& m);

   // Mat assign (const Mat& M);
    //void releaseMemory();
    //void allocateMemory(const int32_t r_, const int32_t c_);

    enum { MAGIC_VAL = 0x42FF0000, AUTO_STEP = 0, CONTINUOUS_FLAG = CV_MAT_CONT_FLAG, SUBMATRIX_FLAG = CV_SUBMAT_FLAG };
    enum { MAGIC_MASK = 0xFFFF0000, TYPE_MASK = 0x00000FFF, DEPTH_MASK = 7 };

    /*! includes several bit-fields:
         - the magic signature
         - continuity flag
         - depth
         - number of channels
     */
    int flags;
    //! the matrix dimensionality, >= 2
    int dims;
    //! the number of rows and columns or (-1, -1) when the matrix has more than 2 dimensions
    int rows, cols;
    //! pointer to the data
    uchar* data;

    //! helper fields used in locateROI and adjustROI
    const uchar* datastart;
    const uchar* dataend;
    const uchar* datalimit;

    //! custom allocator
    int* allocator;
    //! and the standard allocator
 //   static MatAllocator* getStdAllocator();
 //   static MatAllocator* getDefaultAllocator();
 //   static void setDefaultAllocator(MatAllocator* allocator);

    //! internal use method: updates the continuity flag
    void updateContinuityFlag();

    //! interaction with UMat
 //   UMatData* u;
   // int s[2];
    int size[2];
    size_t step[2];

protected:
    template<typename _Tp, typename Functor> void forEach_impl(const Functor& operation);
};

/*Mat& Mat::operator=(const Mat& M)
{
    if (this != &M)
    {
        if (M.rows != rows || M.cols != cols)
        {
            //releaseMemory();
            //allocateMemory(M.rows, M.cols);
           // this->create();
        }
        //this->create();
        static unsigned char ch2[1859 * 1312];

        data = ch2;
        

        int j = 0;
        while (j < 2)//(M.data[j] != NULL)
        {
            cout << "j: " << j << endl;
            data[j] = M.data[j];
            j++;
        }
        data[j] = M.data[j];


    }
    return *this;
}*/

inline
Mat Mat::operator = (const Mat m)
{
    //if (this != &m)
    {
        //if (m.u)
            //CV_XADD(&m.u->refcount, 1);
        //release();
        flags = m.flags;
        if (dims <= 2 && m.dims <= 2)
        {
            dims = m.dims;
            rows = m.rows;
            cols = m.cols;
            step[0] = m.step[0];
            step[1] = m.step[1];
        }
        else
            copySize(m);
        data = m.data;
        datastart = m.datastart;
        dataend = m.dataend;
        datalimit = m.datalimit;
        allocator = m.allocator;
        //u = m.u;
    }
    return *this;
}





/*
void Mat::allocateMemory(const int32_t r_, const int32_t c_)
{
    rows = abs(r_);
    cols = abs(c_);
    if (rows == 0 || cols == 0)
    {
        data = 0;
        return;
    }
   // val = (FLOAT**)malloc(rows * sizeof(FLOAT*));
   // val[0] = (FLOAT*)calloc(rows * cols, sizeof(FLOAT));
   // static unsigned char ch2[ ];

   // data = ch2;
    data = (uchar*)malloc(sizeof(uchar) * (rows+1) * cols);
    //data = new uchar[ (rows + 1) * cols ];
    uchar zero = 0;

    int imcol3 = cols;
    for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            if (data)
                data[k + (j * imcol3)] = zero;
            else
                cout << "In mat.h: allocateMemory error" << endl;
        }
    }
    //for (int32_t i = 1; i < rows; i++)
        //val[i] = val[i - 1] + cols;
}*/

/*void Mat::releaseMemory()
{
    if (data != 0)
    {
        //free(data[0]);
        free(data);
    }
}*/

//#include "Mat.h"



Mat::Mat()
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(0), datastart(0), dataend(0),
    datalimit(0), allocator(0)//, size(&rows)//, step(0)
{
    
    step[0] = 0;
    step[1] = 0;
    size[0] = rows;
        size[1] = cols;

}

inline
Mat::Mat(Size _sz, int _type, void* _data, size_t _step)
    : flags(MAGIC_VAL + (_type & TYPE_MASK)), dims(2), rows(_sz.height), cols(_sz.width),
    data((uchar*)_data), datastart((uchar*)_data), dataend(0), datalimit(0),
    allocator(0)//, size(&rows)
{
   // CV_Assert(total() == 0 || data != NULL);
	size[0] = rows;
	    size[1] = cols;

    size_t esz = CV_ELEM_SIZE(_type), esz1 = CV_ELEM_SIZE1(_type);
    size_t minstep = cols * esz;
    if (_step == AUTO_STEP)
    {
        _step = minstep;
    }
    else
    {
        //CV_DbgAssert(_step >= minstep);

        if (_step % esz1 != 0)
        {
           // CV_Error(Error::BadStep, "Step must be a multiple of esz1");
            cout << "In Mat.h: BadStep" << endl;
        }
    }
    step[0] = _step;
    step[1] = esz;
    datalimit = datastart + _step * rows;
    dataend = datalimit - _step + minstep;
    updateContinuityFlag();
}
/*
Mat::Mat(const Mat& m, const Range& _rowRange, const Range& _colRange)
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(0), datastart(0), dataend(0),
    datalimit(0), allocator(0),  size(&rows)
{
    //CV_Assert(m.dims >= 2);
    if (m.dims > 2)
    {
        cout << "(m.dims > 2)" << endl;
        AutoBuffer__<Range> rs(m.dims);
        rs[0] = _rowRange;
        rs[1] = _colRange;
        for (int i = 2; i < m.dims; i++)
            rs[i] = Range::all();
        *this = m(rs.data());
        return;
    }
    cout << "(m.dims <= 2)" << endl;
    
    *this = m;
    try
    {
        if (_rowRange != Range::all() && _rowRange != Range(0, rows))
        {
            //CV_Assert(0 <= _rowRange.start && _rowRange.start <= _rowRange.end
            //    && _rowRange.end <= m.rows);
            rows = _rowRange.size();
            data += step[0] * _rowRange.start;
            flags |= SUBMATRIX_FLAG;
        }

        if (_colRange != Range::all() && _colRange != Range(0, cols))
        {
            //CV_Assert(0 <= _colRange.start && _colRange.start <= _colRange.end
            //    && _colRange.end <= m.cols);
            cols = _colRange.size();
            data += _colRange.start * elemSize();
            flags |= SUBMATRIX_FLAG;
        }
    }
    catch (...)
    {
        release();
        throw;
    }

    updateContinuityFlag();

    if (rows <= 0 || cols <= 0)
    {
        release();
        rows = cols = 0;
    }
}*/

inline
Mat::~Mat()
{
    /*release();
    if (step.p != step.buf)
        fastFree(step.p);*/
}

/*inline
Mat::Mat(Mat&& m) 
    : flags(m.flags), dims(m.dims), rows(m.rows), cols(m.cols), data(m.data),
    datastart(m.datastart), dataend(m.dataend), datalimit(m.datalimit), allocator(m.allocator),
    size(&rows) 
{
    //if (m.dims <= 2)  // move new step/size info
    //{
        step[0] = m.step[0];
        step[1] = m.step[1];
    //}
    /*else
    {
        //CV_DbgAssert(m.step.p != m.step.buf);
        step.p = m.step.p;
        size.p = m.size.p;
        m.step.p = m.step.buf;
        m.size.p = &m.rows;
    }*/
   /* m.flags = MAGIC_VAL; m.dims = m.rows = m.cols = 0;
    m.data = NULL; m.datastart = NULL; m.dataend = NULL; m.datalimit = NULL;
    m.allocator = NULL;
   // m.u = NULL;
}*/



/*
Mat::Mat(const Mat& m)
    : flags(m.flags), dims(m.dims), rows(m.rows), cols(m.cols), data(m.data),
    datastart(m.datastart), dataend(m.dataend), datalimit(m.datalimit), allocator(m.allocator)//,
    //size(&rows)//, step(0)
{
	size[0] = m.rows;
	    size[1] = m.cols;

    if (m.dims <= 2)
    {
        step[0] = m.step[0]; step[1] = m.step[1];
    }
    else
    {
        dims = 0;
        copySize(m);
        cout << "setsize error: dim > 2" << endl;
    }
}*/

void Mat::copySize(const Mat& m)
{

    for (int i = 0; i < dims; i++)
    {
        size[i] = m.size[i];
        step[i] = m.step[i];
    }
}

inline
int Mat::depth() const
{
    return CV_MAT_DEPTH(flags);
}

inline
int Mat::channels() const
{
    return CV_MAT_CN(flags);
}

inline
int Mat::type() const
{
    return CV_MAT_TYPE(flags);
}

inline
size_t Mat::step1(int i) const
{
    return step[i] / elemSize1();
}


void Mat::locateROI(Size& wholeSize, Point__& ofs) const
{
   // CV_Assert(dims <= 2 && step[0] > 0);
    size_t esz = elemSize(), minstep;
    ptrdiff_t delta1 = data - datastart, delta2 = dataend - datastart;

    if (delta1 == 0)
        ofs.x = ofs.y = 0;
    else
    {
        ofs.y = (int)(delta1 / step[0]);
        ofs.x = (int)((delta1 - step[0] * ofs.y) / esz);
        //CV_DbgAssert(data == datastart + ofs.y * step[0] + ofs.x * esz);
    }
    minstep = (ofs.x + cols) * esz;
    wholeSize.height = (int)((delta2 - minstep) / step[0] + 1);
    wholeSize.height = std::max(wholeSize.height, ofs.y + rows);
    wholeSize.width = (int)((delta2 - step[0] * (wholeSize.height - 1)) / esz);
    wholeSize.width = std::max(wholeSize.width, ofs.x + cols);
}
/*
Mat& Mat::adjustROI(int dtop, int dbottom, int dleft, int dright)
{
    //CV_Assert(dims <= 2 && step[0] > 0);
    Size wholeSize; Point__ ofs;
    size_t esz = elemSize();
    locateROI(wholeSize, ofs);
    int row1 = std::min(std::max(ofs.y - dtop, 0), wholeSize.height), row2 = std::max(0, std::min(ofs.y + rows + dbottom, wholeSize.height));
    int col1 = std::min(std::max(ofs.x - dleft, 0), wholeSize.width), col2 = std::max(0, std::min(ofs.x + cols + dright, wholeSize.width));
    if (row1 > row2)
        std::swap(row1, row2);
    if (col1 > col2)
        std::swap(col1, col2);

    data += (row1 - ofs.y) * step + (col1 - ofs.x) * esz;
    rows = row2 - row1; cols = col2 - col1;
    size.p[0] = rows; size.p[1] = cols;
    updateContinuityFlag();
    return *this;
}*/

Mat::Mat(const Mat m, const Rect& roi)
    : flags(m.flags), dims(2), rows(roi.height), cols(roi.width),
    data(m.data + roi.y * m.step[0]),
    datastart(m.datastart), dataend(m.dataend), datalimit(m.datalimit),
    allocator(m.allocator)//,  size(&rows)
{
   // CV_Assert(m.dims <= 2);
	size[0] = rows;
	    size[1] = cols;

    size_t esz = CV_ELEM_SIZE(flags);
    data += roi.x * esz;
   // CV_Assert(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols &&
   //     0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows);
    //if (u)
    //    ;//CV_XADD(&u->refcount, 1);
    if (roi.width < m.cols || roi.height < m.rows)
        flags |= SUBMATRIX_FLAG;

    step[0] = m.step[0]; step[1] = esz;
    updateContinuityFlag();

    if (rows <= 0 || cols <= 0)
    {
        release();
        rows = cols = 0;
    }
}

/*void Mat::exRect(const Mat m, const Rect roi)
{

    flags = m.flags;
    dims = 2;
    rows = roi.height;
    cols = roi.width;
    data = m.data + roi.y * m.step[0];
    //datastart = m.datastart;
   // dataend = m.dataend;
   // datalimit = m.datalimit;
   // allocator = m.allocator;

    // CV_Assert(m.dims <= 2);
    size[0] = rows;
    size[1] = cols;

    size_t esz = CV_ELEM_SIZE(flags);
    data += roi.x * esz;

    if (roi.width < m.cols || roi.height < m.rows)
        flags |= SUBMATRIX_FLAG;

    step[0] = m.step[0]; step[1] = esz;
    updateContinuityFlag();


}*/

/*
Mat::Mat(const Mat& m, const Range* ranges)
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(0), datastart(0), dataend(0),
    datalimit(0), allocator(0),  size(&rows)
{
    int d = m.dims;

    //CV_Assert(ranges);
    for (int i = 0; i < d; i++)
    {
        Range r = ranges[i];
        //CV_Assert(r == Range::all() || (0 <= r.start && r.start < r.end&& r.end <= m.size[i]));
    }
    *this = m;
    for (int i = 0; i < d; i++)
    {
        Range r = ranges[i];
        if (r != Range::all() && r != Range(0, size[i]))
        {
            size[i] = r.end - r.start;
            data += r.start * step[i];
            flags |= SUBMATRIX_FLAG;
        }
    }
    updateContinuityFlag();
}*/

inline
Mat Mat::operator()(const Rect& roi) const
{
    return Mat(*this, roi);
}

/*inline
Mat Mat::operator()(const Range* ranges) const
{
    return Mat(*this, ranges);
}*/


inline
size_t Mat::elemSize() const
{
    size_t res = dims > 0 ? step[dims - 1] : 0;
    //CV_DbgAssert(res != 0);
    return res;
}

inline
size_t Mat::elemSize1() const
{
    return CV_ELEM_SIZE1(flags);
}

inline
bool Mat::isContinuous() const
{
    return (flags & CONTINUOUS_FLAG) != 0;
}

inline
bool Mat::isSubmatrix() const
{
    return (flags & SUBMATRIX_FLAG) != 0;
}

/*inline
Mat Mat::rowRange(int startrow, int endrow) const
{
    return Mat(*this, Range(startrow, endrow), Range::all());
}*/


inline
size_t Mat::total() const
{
    if (dims <= 2)
        return (size_t)rows * cols;
    size_t p = 1;
    for (int i = 0; i < dims; i++)
        p *= size[i];
    return p;
}

inline
uchar* Mat::ptr(int y)
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return data + step[0] * y;
}

inline
const uchar* Mat::ptr(int y) const
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return data + step[0] * y;
}



template<typename _Tp> inline
_Tp* Mat::ptr(int y)
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (_Tp*)(data + step[0] * y);
}

template<typename _Tp> inline
const _Tp* Mat::ptr(int y) const
{
    //CV_DbgAssert(y == 0 || (data && dims >= 1 && (unsigned)y < (unsigned)size.p[0]));
    return (const _Tp*)(data + step[0] * y);
}


inline
bool Mat::empty() const
{
    return data == 0 || total() == 0 || dims == 0;
}

inline
void Mat::create()
{
    size[0] = 1859;
    size[1] = 1312;
    flags = 1124024320;
    dims = 2;
    step[0] = 1312;
    step[1] = 1;
    rows = 1859;
    cols = 1312;
    static unsigned char ch2[1859 * 1312];

    data = ch2;


    int imcol2 = cols;
    /*for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            data[k + (j * imcol2)] = 205;
        }
    }*/

    datastart = data;
        if (data)
        {
            datalimit = datastart + size[0] * step[0];
            if (size[0] > 0)
            {
                dataend = ptr() + size[dims - 1] * step[dims - 1];
                for (int i = 0; i < dims - 1; i++)
                    dataend += (size[i] - 1) * step[i];
            }
            else
                dataend = datalimit;
        }
        else
            dataend = datalimit = 0;




}


inline
void Mat::createP()
{
    size[0] = 1859;
    size[1] = 1312;
    flags = 1124024320;
    dims = 2;
    step[0] = 1312;
    step[1] = 1;
    rows = 1859;
    cols = 1312;
    unsigned char ch2P[1859 * 1312];

    data = ch2P;


    int imcol2 = cols;
    /*for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            data[k + (j * imcol2)] = 205;
        }
    }*/

    datastart = data;
    if (data)
    {
        datalimit = datastart + size[0] * step[0];
        if (size[0] > 0)
        {
            dataend = ptr() + size[dims - 1] * step[dims - 1];
            for (int i = 0; i < dims - 1; i++)
                dataend += (size[i] - 1) * step[i];
        }
        else
            dataend = datalimit;
    }
    else
        dataend = datalimit = 0;

}

inline
void Mat::create1()
{
    size[0] = 1859;
    size[1] = 1312;
    flags = 1124024320;
    dims = 2;
    step[0] = 1312;
    step[1] = 1;
    rows = 1859;
    cols = 1312;
    static unsigned char ch3[1859 * 1312];

    data = ch3;


    int imcolm = cols;
    /*for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            data[k + (j * imcolm)] = 205;
        }
    }*/
}

inline
void Mat::create_des()
{
    size[0] = 500;
    size[1] = 32;
    flags = 1124024320;
    dims = 2;
    step[0] = 32;
    step[1] = 1;
    rows = 500;
    cols = 32;
    static unsigned char ch_d[500 * 32];

    data = ch_d;


    int imcol_d = cols;
    /*for (int j = 0; j < rows; j++)
    {
        for (int k = 0; k < cols; k++)
        {
            data[k + (j * imcol_d)] = 205;
        }
    }*/
}

inline
void Mat::create_ker()
{
    size[0] = 7;
    size[1] = 1;
    flags = 1124024325;
    dims = 2;
    step[0] = 4;
    step[1] = 4;
    rows = 7;
    cols = 1;
    static unsigned char ch_k[7 * 4];

    data = ch_k;


    for (int j = 0; j < 28; j++)
       {

          //at<float>(j, 0) = -431602080.000000;
           data[j] = 205;

       }

}

/*
inline
void Mat::create(int _rows, int _cols, int _type)
{
    _type &= TYPE_MASK;
    if (dims <= 2 && rows == _rows && cols == _cols && type() == _type && data)
        return;
    int sz[] = { _rows, _cols };
    create(2, sz, _type);
}

void finalizeHdr(Mat& m)
{
    m.updateContinuityFlag();
    int d = m.dims;
    if (d > 2)
        m.rows = m.cols = -1;
    if (m.u)
        m.datastart = m.data = m.u->data;
    if (m.data)
    {
        m.datalimit = m.datastart + m.size[0] * m.step[0];
        if (m.size[0] > 0)
        {
            m.dataend = m.ptr() + m.size[d - 1] * m.step[d - 1];
            for (int i = 0; i < d - 1; i++)
                m.dataend += (m.size[i] - 1) * m.step[i];
        }
        else
            m.dataend = m.datalimit;
    }
    else
        m.dataend = m.datalimit = 0;
}*/


/*
void Mat::create(int d, const int* _sizes, int _type)
{
    int i;
    //CV_Assert(0 <= d && d <= CV_MAX_DIM && _sizes);
    //_type = CV_MAT_TYPE(_type);

    if (data && (d == dims || (d == 1 && dims <= 2)) && _type == type())
    {
        if (d == 2 && rows == _sizes[0] && cols == _sizes[1])
            return;
        for (i = 0; i < d; i++)
            if (size[i] != _sizes[i])
                break;
        if (i == d && (d > 1 || size[1] == 1))
            return;
    }

    int _sizes_backup[CV_MAX_DIM]; // #5991
    if (_sizes == (this->size.p))
    {
        for (i = 0; i < d; i++)
            _sizes_backup[i] = _sizes[i];
        _sizes = _sizes_backup;
    }

    release();
    if (d == 0)
        return;
    flags = (_type & CV_MAT_TYPE_MASK) | MAGIC_VAL;
    setSize(*this, d, _sizes, 0, true);

    if (total() > 0)
    {
        MatAllocator* a = allocator, * a0 = getDefaultAllocator();
#ifdef HAVE_TGPU
        if (!a || a == tegra::getAllocator())
            a = tegra::getAllocator(d, _sizes, _type);
#endif
        if (!a)
            a = a0;
        try
        {
            u = a->allocate(dims, size, _type, 0, step.p, ACCESS_RW, USAGE_DEFAULT);
            CV_Assert(u != 0);
        }
        catch (...)
        {
            if (a == a0)
                throw;
            u = a0->allocate(dims, size, _type, 0, step.p, ACCESS_RW, USAGE_DEFAULT);
            CV_Assert(u != 0);
        }
        CV_Assert(step[dims - 1] == (size_t)CV_ELEM_SIZE(flags));
    }

    addref();
    finalizeHdr(*this);
}*/

/*
void fastFree(void* ptr)
{

    if (ptr)
    {
        uchar* udata = ((uchar**)ptr)[-1];
        //CV_DbgAssert(udata < (uchar*)ptr &&
        //    ((uchar*)ptr - udata) <= (ptrdiff_t)(sizeof(void*) + CV_MALLOC_ALIGN));
        free(udata);
    }
}
*/

inline
void Mat::release()
{
    /*if (u && CV_XADD(&u->refcount, -1) == 1)
        deallocate();
    u = NULL;*/
    datastart = dataend = datalimit = data = 0;
    for (int i = 0; i < dims; i++)
        size[i] = 0;
//#ifdef _DEBUG
    flags = MAGIC_VAL;
    dims = rows = cols = 0;
    //if (step.p != step.buf)
    //{
        //fastFree(step.p);
        step[0] = 0;
        step[1] = 0;
        //size = &rows;
        size[0] = rows;
                size[1] = cols;

    //}
//#endif
}

/*
inline
Mat& Mat::operator = (Mat&& m) 
{
    if (this == &m)
      return *this;

    release();
    flags = m.flags; dims = m.dims; rows = m.rows; cols = m.cols; data = m.data;
    datastart = m.datastart; dataend = m.dataend; datalimit = m.datalimit; allocator = m.allocator;
   // u = m.u;

    //if (m.dims <= 2) // move new step/size info
    //{
        step[0] = m.step[0];
        step[1] = m.step[1];
    //}

    m.flags = MAGIC_VAL; m.dims = m.rows = m.cols = 0;
    m.data = NULL; m.datastart = NULL; m.dataend = NULL; m.datalimit = NULL;
    m.allocator = NULL;
    //m.u = NULL;
    return *this;
}*/



int updateContinuityFlag1(int flags, int dims, const int* size, const size_t* step)
{
    int i, j;
    for (i = 0; i < dims; i++)
    {
        if (size[i] > 1)
            break;
    }

    uint64 t = (uint64)size[std::min(i, dims - 1)] * CV_MAT_CN(flags);
    for (j = dims - 1; j > i; j--)
    {
        t *= size[j];
        if (step[j] * size[j] < step[j - 1])
            break;
    }

    if (j <= i && t == (uint64)(int)t)
        return flags | Mat::CONTINUOUS_FLAG;
    return flags & ~Mat::CONTINUOUS_FLAG;
}

void Mat::updateContinuityFlag()
{
    flags = updateContinuityFlag1(flags, dims, size, step);
}

template<typename _Tp> inline
_Tp& Mat::at(int i0, int i1)
{
    //CV_DbgAssert(dims <= 2);
    //CV_DbgAssert(data);
    //CV_DbgAssert((unsigned)i0 < (unsigned)size.p[0]);
    //CV_DbgAssert((unsigned)(i1 * DataType<_Tp>::channels) < (unsigned)(size.p[1] * channels()));
   // CV_DbgAssert(CV_ELEM_SIZE1(traits::Depth<_Tp>::value) == elemSize1());
    return ((_Tp*)(data + step[0] * i0))[i1];
}

template<typename _Tp> inline
const _Tp& Mat::at(int i0, int i1) const
{
    //CV_DbgAssert(dims <= 2);
   // CV_DbgAssert(data);
    //CV_DbgAssert((unsigned)i0 < (unsigned)size.p[0]);
    //CV_DbgAssert((unsigned)(i1 * DataType<_Tp>::channels) < (unsigned)(size.p[1] * channels()));
    //CV_DbgAssert(CV_ELEM_SIZE1(traits::Depth<_Tp>::value) == elemSize1());
    return ((const _Tp*)(data + step[0] * i0))[i1];
}

template<typename _Tp> inline
_Tp& Mat::at(int i0)
{
    //CV_DbgAssert(dims <= 2);
    //CV_DbgAssert(data);
    //CV_DbgAssert((unsigned)i0 < (unsigned)(size.p[0] * size.p[1]));
    //CV_DbgAssert(elemSize() == sizeof(_Tp));
    if (isContinuous() || size[0] == 1)
        return ((_Tp*)data)[i0];
    if (size[1] == 1)
        return *(_Tp*)(data + step[0] * i0);
    int i = i0 / cols, j = i0 - i * cols;
    return ((_Tp*)(data + step[0] * i))[j];
}

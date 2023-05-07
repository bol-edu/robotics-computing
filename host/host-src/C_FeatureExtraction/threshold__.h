
//#include <limits>
//#include <cstddef>
//#include "parallel_for__.h"
using namespace std;


enum
{
    //CV_THRESH_BINARY = 0,  /**< value = value > threshold ? max_value : 0       */
    CV_THRESH_BINARY_INV = 1,  /**< value = value > threshold ? 0 : max_value       */
    CV_THRESH_TRUNC = 2,  /**< value = value > threshold ? threshold : value   */
    CV_THRESH_TOZERO = 3,  /**< value = value > threshold ? value : 0           */
    CV_THRESH_TOZERO_INV = 4,  /**< value = value > threshold ? 0 : value           */
    CV_THRESH_MASK = 7,
    CV_THRESH_OTSU = 8, /**< use Otsu algorithm to choose the optimal threshold value;
                                 combine the flag with one of the above CV_THRESH_* values */
    CV_THRESH_TRIANGLE = 16  /**< use Triangle algorithm to choose the optimal threshold value;
                                                              combine the flag with one of the above CV_THRESH_* values, but not
                                                              with CV_THRESH_OTSU */
};

enum ThresholdTypes_ {
    THRESH_BINARY_ = 0, //!< \f[\texttt{dst} (x,y) =  \fork{\texttt{maxval}}{if \(\texttt{src}(x,y) > \texttt{thresh}\)}{0}{otherwise}\f]
    THRESH_BINARY_INV_ = 1, //!< \f[\texttt{dst} (x,y) =  \fork{0}{if \(\texttt{src}(x,y) > \texttt{thresh}\)}{\texttt{maxval}}{otherwise}\f]
    THRESH_TRUNC_ = 2, //!< \f[\texttt{dst} (x,y) =  \fork{\texttt{threshold}}{if \(\texttt{src}(x,y) > \texttt{thresh}\)}{\texttt{src}(x,y)}{otherwise}\f]
    THRESH_TOZERO_ = 3, //!< \f[\texttt{dst} (x,y) =  \fork{\texttt{src}(x,y)}{if \(\texttt{src}(x,y) > \texttt{thresh}\)}{0}{otherwise}\f]
    THRESH_TOZERO_INV_ = 4, //!< \f[\texttt{dst} (x,y) =  \fork{0}{if \(\texttt{src}(x,y) > \texttt{thresh}\)}{\texttt{src}(x,y)}{otherwise}\f]
    THRESH_MASK_ = 7,
    THRESH_OTSU_ = 8, //!< flag, use Otsu algorithm to choose the optimal threshold value
    THRESH_TRIANGLE_ = 16 //!< flag, use Triangle algorithm to choose the optimal threshold value
};





static void
thresh_8u(const Mat& _src, Mat& _dst, uchar thresh, uchar maxval, int type)
{
    Size src_size(_src.size[1], _src.size[0]);
    Size roi = src_size;
    //roi.width *= _src.channels();
    size_t src_step = _src.step[0];
    size_t dst_step = _dst.step[0];

    if (_src.isContinuous() && _dst.isContinuous())
    {
        roi.width *= roi.height;
        roi.height = 1;
        src_step = dst_step = roi.width;
    }



    int j = 0;
    const uchar* src = _src.ptr();
    uchar* dst = _dst.ptr();


    int j_scalar = j;
    if (j_scalar < roi.width)
    {
        const int thresh_pivot = thresh + 1;
        uchar tab[256] = { 0 };
        switch (type)
        {
        case THRESH_BINARY_:
            memset(tab, 0, thresh_pivot);
            if (thresh_pivot < 256) {
                memset(tab + thresh_pivot, maxval, 256 - thresh_pivot);
            }
            break;
        case THRESH_BINARY_INV_:
            memset(tab, maxval, thresh_pivot);
            if (thresh_pivot < 256) {
                memset(tab + thresh_pivot, 0, 256 - thresh_pivot);
            }
            break;
        case THRESH_TRUNC_:
            for (int i = 0; i <= thresh; i++)
                tab[i] = (uchar)i;
            if (thresh_pivot < 256) {
                memset(tab + thresh_pivot, thresh, 256 - thresh_pivot);
            }
            break;
        case THRESH_TOZERO_:
            memset(tab, 0, thresh_pivot);
            for (int i = thresh_pivot; i < 256; i++)
                tab[i] = (uchar)i;
            break;
        case THRESH_TOZERO_INV_:
            for (int i = 0; i <= thresh; i++)
                tab[i] = (uchar)i;
            if (thresh_pivot < 256) {
                memset(tab + thresh_pivot, 0, 256 - thresh_pivot);
            }
            break;
        }

        src = _src.ptr();
        dst = _dst.ptr();
        for (int i = 0; i < roi.height; i++, src += src_step, dst += dst_step)
        {
//#pragma HLS loop_tripcount max=313
#pragma HLS PIPELINE
            j = j_scalar;
/*#if CV_ENABLE_UNROLLED
            for (; j <= roi.width - 4; j += 4)
            {
                uchar t0 = tab[src[j]];
                uchar t1 = tab[src[j + 1]];

                dst[j] = t0;
                dst[j + 1] = t1;

                t0 = tab[src[j + 2]];
                t1 = tab[src[j + 3]];

                dst[j + 2] = t0;
                dst[j + 3] = t1;
            }
#endif*/
            for (; j < roi.width; j++){
//#pragma HLS loop_tripcount max=1034
#pragma HLS PIPELINE
#pragma HLS unroll factor=4
                dst[j] = tab[src[j]];}
        }
    }
}








double threshold__(Mat src, Mat& dst, double thresh, double maxval, int type)
{
    //CV_INSTRUMENT_REGION();

   // CV_OCL_RUN_(_src.dims() <= 2 && _dst.isUMat(),
    //    ocl_threshold(_src, _dst, thresh, maxval, type), thresh)

        //Mat src = _src.getMat();
    int automatic_thresh = (type & ~CV_THRESH_MASK);
    type &= THRESH_MASK_;
   


    //_dst.create(src.size(), src.type());
   // Mat dst = _dst.getMat();

    //cout << " dep: " << src.depth() << endl; == 0

  //  if (src.depth() == 0)  // if (src.depth() == CV_8U)  (src.depth() : 0)
    {
        int ithresh = cvFloor(thresh);
        thresh = ithresh;
        int imaxval = cvRound(maxval);
        if (type == THRESH_TRUNC_)
            imaxval = ithresh;
        imaxval = saturate_cast<uchar>(imaxval);

        if (ithresh < 0 || ithresh >= 255)
        {
            //cout << "In threshold__.h: ithresh < 0 || ithresh >= 255" << endl;
            /*if (type == THRESH_BINARY_ || type == THRESH_BINARY_INV_ ||
                ((type == THRESH_TRUNC_ || type == THRESH_TOZERO_INV_) && ithresh < 0) ||
                (type == THRESH_TOZERO_ && ithresh >= 255))
            {
                int v = type == THRESH_BINARY_ ? (ithresh >= 255 ? 0 : imaxval) :
                    type == THRESH_BINARY_INV_ ? (ithresh >= 255 ? imaxval : 0) :
                    0;
                dst.setTo(v);
            }
            else
                src.copyTo(dst);
            return thresh;*/
        }



        thresh = ithresh;
        maxval = imaxval;
    }

 /*   else if (src.depth() == 5)
        ;
    else if (src.depth() == 6)
        ;
    else
        //CV_Error(CV_StsUnsupportedFormat, "");
        ;*/

    Range range = Range(0, dst.rows);


            int row0 = range.start;
            int row1 = range.end;

            Mat srcStripe = src;//.rowRange(row0, row1);
            Mat dstStripe = dst;// .rowRange(row0, row1);


           // cout << "ssdep: " << srcStripe.depth() << endl;  0
           // if (srcStripe.depth() == CV_8U)
           // {
                thresh_8u(srcStripe, dstStripe, (uchar)thresh, (uchar)maxval, type);
           // }



   // parallel_for__(Range(0, dst.rows),
    //    ThresholdRunner(src, dst, thresh, maxval, type),
    //    dst.total() / (double)(1 << 16));
    return thresh;
}

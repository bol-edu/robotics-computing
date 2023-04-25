
# include <stdio.h>
# include <iostream>
# include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <algorithm>
#include "Mat.h"
#include "Methods.h"
#include "hls_stream.h"

using namespace std;

#define small_width 414
#define small_height 376

#define whole_width 1241
#define whole_height 376

#define CV_32F  5
#define CV_SIMD CV_SIMD128
#define CV_SIMD_64F CV_SIMD128_64F
#define CV_SIMD_WIDTH 16
typedef uchar PixType;
typedef short CostType;
typedef short DispType;
typedef unsigned short ushort;
enum {
    DISP_SHIFT = 4,
    DISP_SCALE = (1 << DISP_SHIFT)
};

enum { NR = 8, NR2 = NR / 2 };

unsigned char Mat_buffer[4 * small_width * small_height];


//alignSize---------------------------------------------------------------------------//
static inline size_t alignSize__(size_t sz, int n)
{
    return (sz + n - 1) & -n;
    //cout<<"alignsize"<<endl;
}

static void calcPixelCostBT__(const Mat& img1, const Mat& img2, int y,
    int minD, int maxD, CostType* cost,
    PixType* buffer, const PixType* tab,
    int xrange_min = 0, int xrange_max = -1
)
{
    //cout<<"calcPixelCostBT"<<endl;
    int x, c, width = img1.cols, cn = img1.channels();
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    int D = (int)alignSize__(maxD - minD, 8), width1 = maxX1 - minX1;
    xrange_min = (xrange_min < 0) ? 0 : xrange_min;
    xrange_max = (xrange_max == -1) || (xrange_max > width1) ? width1 : xrange_max;
    maxX1 = minX1 + xrange_max;
    minX1 += xrange_min;
    width1 = maxX1 - minX1;
    int minX2 = max(minX1 - maxD, 0), maxX2 = min(maxX1 - minD, width);
    int width2 = maxX2 - minX2;
    const PixType* row1 = img1.ptr<PixType>(y), * row2 = img2.ptr<PixType>(y);
    PixType* prow1 = buffer + width2 * 2, * prow2 = prow1 + width * cn * 2;

    for (c = 0; c < cn * 2; c++)
    {
#pragma HLS PIPELINE
        prow1[width * c] = tab[0];
        prow1[width * c + width - 1] = tab[0];
        prow2[width * c] = tab[0];
        prow2[width * c + width - 1] = tab[0];
    }

    int n1 = y > 0 ? -(int)img1.step[0] : 0, s1 = y < img1.rows - 1 ? (int)img1.step[0] : 0;
    int n2 = y > 0 ? -(int)img2.step[0] : 0, s2 = y < img2.rows - 1 ? (int)img2.step[0] : 0;

    int minX_cmn = min(minX1, minX2) - 1;
    int maxX_cmn = max(maxX1, maxX2) + 1;
    minX_cmn = max(minX_cmn, 1);
    maxX_cmn = min(maxX_cmn, width - 1);

    for (x = minX_cmn; x < maxX_cmn; x++)
    {
#pragma HLS PIPELINE
        prow1[x] = tab[(row1[x + 1] - row1[x - 1]) * 2 + row1[x + n1 + 1] - row1[x + n1 - 1] + row1[x + s1 + 1] - row1[x + s1 - 1]];
        prow2[width - 1 - x] = tab[(row2[x + 1] - row2[x - 1]) * 2 + row2[x + n2 + 1] - row2[x + n2 - 1] + row2[x + s2 + 1] - row2[x + s2 - 1]];

        prow1[x + width] = row1[x];
        prow2[width - 1 - x + width] = row2[x];
    }

    memset(cost + xrange_min * D, 0, width1 * D * sizeof(cost[0]));

    buffer -= width - maxX2;
    //uchar buffer1[1241+1241+96];
    cost -= (minX1 - xrange_min) * D + minD; // simplify the cost indices inside the loop

    for (c = 0; c < cn * 2; c++, prow1 += width, prow2 += width)
    {
        int diff_scale = c < cn ? 0 : 2;

        // precompute
        //   v0 = min(row2[x-1/2], row2[x], row2[x+1/2]) and
        //   v1 = max(row2[x-1/2], row2[x], row2[x+1/2]) and
        //   to process values from [minX2, maxX2) we should check memory location (width - 1 - maxX2, width - 1 - minX2]
        //   so iterate through [width - maxX2, width - minX2)
        for (x = width - maxX2; x < width - minX2; x++)
        {
#pragma HLS UNROLL
            int vtemp0 = prow2[x - 1];
            int vtemp1 = prow2[x + 1];
            int v = prow2[x];
            int vl = x > 0 ? (v + vtemp0) / 2 : v;
            int vr = x < width - 1 ? (v + vtemp1) / 2 : v;
            int v0 = min(vl, vr); v0 = min(v0, v);
            int v1 = max(vl, vr); v1 = max(v1, v);
            buffer[x] = (PixType)v0;
            buffer[x + width2] = (PixType)v1;
        }

        for (x = minX1; x < maxX1; x++)
        {

            //int utemp0 = prow1[x - 1];
            //int utemp1 = prow1[x + 1];
            int u = prow1[x];
            int ul = x > 0 ? (u + prow1[x - 1]) / 2 : u;
            int ur = x < width - 1 ? (u + prow1[x + 1]) / 2 : u;
            int u0 = min(ul, ur); u0 = min(u0, u);
            int u1 = max(ul, ur); u1 = max(u1, u);

            int d = minD;

            for (; d < maxD; d++)
            {
#pragma HLS UNROLL
                int v = prow2[width - x - 1 + d];
                int v0 = buffer[width - x - 1 + d];
                int v1 = buffer[width - x - 1 + d + width2];
                int c0 = max(0, u - v1); c0 = max(c0, v0 - u);
                int c1 = max(0, v - u1); c1 = max(c1, u0 - v);

                cost[x * D + d] = (CostType)(cost[x * D + d] + (min(c0, c1) >> diff_scale));
            }
        }
    }
}

//Range----------------------------------------------------------------------
class  Range__
{
public:

    Range__(int _start, int _end);
    int size() const;
    static Range__ all();

    int start, end;
};

inline
Range__::Range__(int _start, int _end)
    : start(_start), end(_end) {}

inline
int Range__::size() const
{
    return end - start;
}

inline
Range__ Range__::all()
{
    return Range__(INT_MIN, INT_MAX);
}

static inline
bool operator == (const Range__& r1, const Range__& r2)
{
    return r1.start == r2.start && r1.end == r2.end;
}

//---------------------------------------------------------------------
class BufferSGBM__
{
private:
    size_t width1;
    size_t Da;
    size_t Dlra;
    size_t costWidth;
    size_t costHeight;
    size_t hsumRows;
    bool fullDP;
    uchar dirs;
    uchar dirs2;
    static const size_t TAB_OFS = 256 * 4;

public:
    CostType* Cbuf;
    CostType* Sbuf;
    CostType* hsumBuf;
    CostType* pixDiff;
    CostType* disp2cost;
    DispType* disp2ptr;
    PixType* tempBuf;
    short* Lr_new0;
    short* Lr_new1;
    short* minLr_new0;
    short* minLr_new1;
    PixType* clipTab;

public:
    BufferSGBM__(size_t width1_,
        size_t Da_,
        size_t Dlra_,
        size_t cn,
        size_t width,
        size_t height
    )
        : width1(width1_),
        Da(Da_),
        Dlra(Dlra_),
        Cbuf(NULL),
        Sbuf(NULL),
        hsumBuf(NULL),
        pixDiff(NULL),
        disp2cost(NULL),
        disp2ptr(NULL),
        tempBuf(NULL),
        clipTab(NULL)
    {
        const size_t TAB_SIZE = 256 + TAB_OFS * 2;
        fullDP = 0;
        costWidth = width1 * Da;
        costHeight = fullDP ? height : 1;
        int ps;
        ps = 11;
        hsumRows = ps + 2;//hsumRows = params.calcSADWindowSize().height + 2;
        dirs = NR;
        dirs2 = NR2;

        //allocate-----------------------------------
        static short Cbuf_new[(small_width - 96) * 96];
        Cbuf = Cbuf_new;

        static short Sbuf_new[(small_width - 96) * 96];
        Sbuf = Sbuf_new;

        static short hsumBuf_new[(small_width - 96) * 96 * 13];
        hsumBuf = hsumBuf_new;

        static short pixDiff_new[(small_width - 96) * 96 + 5000];
        pixDiff = pixDiff_new;

        static short disp2cost_new[small_width];
        disp2cost = disp2cost_new;

        static short disp2ptr_new[small_width];
        disp2ptr = disp2ptr_new;

        static uchar tempBuf_new[small_width * 6];
        tempBuf = tempBuf_new;

        static short Lr_new0_new[((small_width - 96) * 4 + 2 * 8) * 104];
        Lr_new0 = Lr_new0_new;

        static short minLr_new0_new[(small_width - 96) * 4 + 2 * 8];
        minLr_new0 = minLr_new0_new;

        static short Lr_new1_new[((small_width - 96) * 4 + 2 * 8) * 104];
        Lr_new1 = Lr_new1_new;

        static short minLr_new1_new[(small_width - 96) * 4 + 2 * 8];
        minLr_new1 = minLr_new1_new;

        static uchar clipTab_new[2304];
        clipTab = clipTab_new;



        // init clipTab
        const int ftzero = 15 | 1;
        for (int i = 0; i < (int)TAB_SIZE; i++)
            clipTab[i] = (PixType)(min(max(i - (int)TAB_OFS, -ftzero), ftzero) + ftzero);
    }
    inline const PixType* getClipTab() const
    {
        return clipTab + TAB_OFS;
    }
    inline void initCBuf(CostType val) const
    {
        for (size_t i = 0; i < costWidth * costHeight; ++i)
            Cbuf[i] = val;
    }
    inline void clearLr(const Range__& range = Range__::all()) const
    {

        //if (range == Range__::all()) {
        memset(Lr_new0, 0, calcLrCount() * Dlra * sizeof(CostType));
        memset(minLr_new0, 0, calcLrCount() * sizeof(CostType));
        /*} else {
                memset(getLr(0, range.start), 0, range.size() * sizeof(CostType) * Dlra);
                memset(getMinLr(0, range.start), 0, range.size() * sizeof(CostType));
        }*/

        //if (range == Range__::all()) {
        memset(Lr_new1, 0, calcLrCount() * Dlra * sizeof(CostType));
        memset(minLr_new1, 0, calcLrCount() * sizeof(CostType));
        /*} else {
                memset(getLr(1, range.start), 0, range.size() * sizeof(CostType) * Dlra);
                memset(getMinLr(1, range.start), 0, range.size() * sizeof(CostType));
        }*/

    }
    inline size_t calcLrCount() const
    {
        return width1 * dirs2 + 2 * dirs;
    }
    inline CostType* getHSumBuf(int row) const
    {
        return hsumBuf + (row % hsumRows) * costWidth;
    }
    inline CostType* getCBuf(int row) const
    {
        return Cbuf + (!fullDP ? 0 : (row * costWidth));
    }
    inline CostType* getSBuf(int row) const
    {
        return Sbuf + (!fullDP ? 0 : (row * costWidth));
    }
    inline void clearSBuf(int row, const Range__& range = Range__::all()) const
    {
        //if (range == Range__::all())
        memset(getSBuf(row), 0, costWidth * sizeof(CostType));
        /*else
            memset(getSBuf(row) + range.start * Da, 0, range.size() * Da * sizeof(CostType));*/
    }

    inline CostType* getLr(uchar id, int idx, uchar shift = 0) const
    {

        const size_t fixed_offset = dirs * Dlra;

        if (id == 0) {
            return Lr_new0 + fixed_offset + (idx * (int)dirs2 + (int)shift) * (int)Dlra;
        }
        else {
            return Lr_new1 + fixed_offset + (idx * (int)dirs2 + (int)shift) * (int)Dlra;
        }

    }
    inline CostType* getMinLr(uchar id, int idx, uchar shift = 0) const
    {
        const size_t fixed_offset = dirs;

        if (id == 0) {
            return minLr_new0 + fixed_offset + (idx * dirs2 + shift);
        }
        else {
            return minLr_new1 + fixed_offset + (idx * dirs2 + shift);
        }
    }
};

template<typename _Tp> static inline _Tp saturate_cast__(int v) { return _Tp(v); }
template<> inline short saturate_cast__<short>(int v) { return (short)((unsigned)(v - SHRT_MIN) <= (unsigned)USHRT_MAX ? v : v > 0 ? SHRT_MAX : SHRT_MIN); }

static void computeDisparitySGBM__(const Mat& img1, const Mat& img2,
    Mat& disp1)
{
    int minDisparity = 0;
    int numDisparities = 96;
    int SADWindowSize = 11;
    int P1 = 864;
    int P2 = 3456;
    int preFilterCap = 0;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = 0;

    const int DISP_SHIFT = 4;
    const int DISP_SCALE = (1 << DISP_SHIFT);
    const CostType MAX_COST = SHRT_MAX;

    int minD = minDisparity, maxD = minD + numDisparities;
    int disp12MaxDiff = 1;
    int k, width = disp1.cols, height = disp1.rows;
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    const int D = numDisparities;
    int width1 = maxX1 - minX1;
    int Da = (int)alignSize__(D, 8);
    int Dlra = Da + 8;//Additional memory is necessary to store disparity values(MAX_COST) for d=-1 and d=D
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP * DISP_SCALE;
    int ps2;
    ps2 = SADWindowSize > 0 ? SADWindowSize : 5;
    int SW2 = ps2 / 2, SH2 = ps2 / 2;
    int npasses = 1;

    //cout<<"computeDisparitySGBM"<<endl;
    BufferSGBM__ mem(width1, Da, Dlra, img1.channels(), width, height);
    mem.initCBuf((CostType)P2); // add P2 to every C(x,y). it saves a few operations in the inner loops

    //for (int pass = 1; pass <= npasses; pass++)
    //{
    int x1, y1, x2, y2, dx, dy;

    //if (pass == 1)
    //{
    y1 = 0; y2 = height; dy = 1;
    x1 = 0; x2 = width1; dx = 1;
    //}
    /*else
    {
        y1 = height - 1; y2 = -1; dy = -1;
        x1 = width1 - 1; x2 = -1; dx = -1;
    }*/

    uchar lrID = 0;
    mem.clearLr();

    for (int y = y1; y != y2; y += dy)
    {
        int x, d;
        DispType* disp1ptr = disp1.ptr<DispType>(y);
        CostType* const C = mem.getCBuf(y);
        CostType* const S = mem.getSBuf(y);

        //if (pass == 1) // compute C on the first pass, and reuse it on the second pass, if any.
        //{
        int dy1 = y == 0 ? 0 : y + SH2, dy2 = y == 0 ? SH2 : dy1;

        for (k = dy1; k <= dy2; k++)
        {

            CostType* hsumAdd = mem.getHSumBuf(min(k, height - 1));

            if (k < height)
            {
                calcPixelCostBT__(img1, img2, k, minD, maxD, mem.pixDiff, mem.tempBuf, mem.getClipTab());

                memset(hsumAdd, 0, Da * sizeof(CostType));

                for (d = 0; d < D; d++)
                {
                    hsumAdd[d] = (CostType)(mem.pixDiff[d] * (SW2 + 1));
                    for (x = Da; x <= SW2 * Da; x += Da)
                        hsumAdd[d] = (CostType)(hsumAdd[d] + mem.pixDiff[x + d]);
                }


                if (y > 0)
                {
                    const CostType* hsumSub = mem.getHSumBuf(max(y - SH2 - 1, 0));
                    const CostType* Cprev = mem.getCBuf(y - 1);


                    for (d = 0; d < D; d++)
                        C[d] = (CostType)(Cprev[d] + hsumAdd[d] - hsumSub[d]);


                    for (x = Da; x < width1 * Da; x += Da)
                    {
                        const CostType* pixAdd = mem.pixDiff + min(x + SW2 * Da, (width1 - 1) * Da);
                        const CostType* pixSub = mem.pixDiff + max(x - (SW2 + 1) * Da, 0);


                        for (d = 0; d < D; d++)
                        {
                            int hv = hsumAdd[x + d] = (CostType)(hsumAdd[x - Da + d] + pixAdd[d] - pixSub[d]);
                            C[x + d] = (CostType)(Cprev[x + d] + hv - hsumSub[x + d]);
                        }

                    }
                }
                else
                {


                    int scale = k == 0 ? SH2 + 1 : 1;
                    for (d = 0; d < D; d++)
                        C[d] = (CostType)(C[d] + hsumAdd[d] * scale);

                    for (x = Da; x < width1 * Da; x += Da)
                    {
                        const CostType* pixAdd = mem.pixDiff + min(x + SW2 * Da, (width1 - 1) * Da);
                        const CostType* pixSub = mem.pixDiff + max(x - (SW2 + 1) * Da, 0);


                        for (d = 0; d < D; d++)
                        {
                            CostType hv = (CostType)(hsumAdd[x - Da + d] + pixAdd[d] - pixSub[d]);
                            hsumAdd[x + d] = hv;
                            C[x + d] = (CostType)(C[x + d] + hv * scale);
                        }

                    }
                }
            }
            else
            {
                //if (y > 0)
                //{
                const CostType* hsumSub = mem.getHSumBuf(max(y - SH2 - 1, 0));
                const CostType* Cprev = mem.getCBuf(y - 1);

                for (x = 0; x < width1 * Da; x++)
                    C[x] = (CostType)(Cprev[x] + hsumAdd[x] - hsumSub[x]);
                //}
                //else
                //{
//
                         //   for (x = 0; x < width1 * Da; x++)
                         //       C[x] = (CostType)(C[x] + hsumAdd[x]);
                        //}
            }

        }

        mem.clearSBuf(y);
        //}


        for (x = x1; x != x2; x += dx)
        {

            int delta0 = P2 + *mem.getMinLr(lrID, x - dx);
            int delta1 = P2 + *mem.getMinLr(1 - lrID, x - 1, 1);
            int delta2 = P2 + *mem.getMinLr(1 - lrID, x, 2);
            int delta3 = P2 + *mem.getMinLr(1 - lrID, x + 1, 3);

            CostType* Lr_p0 = mem.getLr(lrID, x - dx);
            CostType* Lr_p1 = mem.getLr(1 - lrID, x - 1, 1);
            CostType* Lr_p2 = mem.getLr(1 - lrID, x, 2);
            CostType* Lr_p3 = mem.getLr(1 - lrID, x + 1, 3);

            Lr_p0[-1] = Lr_p0[D] = MAX_COST;
            Lr_p1[-1] = Lr_p1[D] = MAX_COST;
            Lr_p2[-1] = Lr_p2[D] = MAX_COST;
            Lr_p3[-1] = Lr_p3[D] = MAX_COST;

            CostType* Lr_p = mem.getLr(lrID, x);
            const CostType* Cp = C + x * Da;
            CostType* Sp = S + x * Da;

            CostType* minL = mem.getMinLr(lrID, x);
            d = 0;

            minL[0] = MAX_COST;
            minL[1] = MAX_COST;
            minL[2] = MAX_COST;
            minL[3] = MAX_COST;

            for (; d < D; d++)
            {

                int Cpd = Cp[d], L;
                int Spd = Sp[d];

                L = Cpd + min((int)Lr_p0[d], min(Lr_p0[d - 1] + P1, min(Lr_p0[d + 1] + P1, delta0))) - delta0;
                Lr_p[d] = (CostType)L;
                minL[0] = min(minL[0], (CostType)L);
                Spd += L;

                L = Cpd + min((int)Lr_p1[d], min(Lr_p1[d - 1] + P1, min(Lr_p1[d + 1] + P1, delta1))) - delta1;
                Lr_p[d + Dlra] = (CostType)L;
                minL[1] = min(minL[1], (CostType)L);
                Spd += L;

                L = Cpd + min((int)Lr_p2[d], min(Lr_p2[d - 1] + P1, min(Lr_p2[d + 1] + P1, delta2))) - delta2;
                Lr_p[d + Dlra * 2] = (CostType)L;
                minL[2] = min(minL[2], (CostType)L);
                Spd += L;

                L = Cpd + min((int)Lr_p3[d], min(Lr_p3[d - 1] + P1, min(Lr_p3[d + 1] + P1, delta3))) - delta3;
                Lr_p[d + Dlra * 3] = (CostType)L;
                minL[3] = min(minL[3], (CostType)L);
                Spd += L;

                Sp[d] = saturate_cast__<CostType>(Spd);
            }
        }

        //if (pass == npasses)
        //{
        x = 0;

        for (; x < width; x++)
        {
            disp1ptr[x] = mem.disp2ptr[x] = (DispType)INVALID_DISP_SCALED;
            mem.disp2cost[x] = MAX_COST;
        }

        for (x = width1 - 1; x >= 0; x--)
        {
            CostType* Sp = S + x * Da;
            CostType minS = MAX_COST;
            short bestDisp = -1;

            //if (npasses == 1)
            //{
            CostType* Lr_p0 = mem.getLr(lrID, x + 1);
            Lr_p0[-1] = Lr_p0[D] = MAX_COST;
            CostType* Lr_p = mem.getLr(lrID, x);

            const CostType* Cp = C + x * Da;

            d = 0;
            int delta0 = P2 + *mem.getMinLr(lrID, x + 1);
            int minL0 = MAX_COST;

            for (; d < D; d++)
            {
                int L0 = Cp[d] + min((int)Lr_p0[d], min(Lr_p0[d - 1] + P1, min(Lr_p0[d + 1] + P1, delta0))) - delta0;

                Lr_p[d] = (CostType)L0;
                minL0 = min(minL0, L0);

                CostType Sval = Sp[d] = saturate_cast__<CostType>(Sp[d] + L0);
                if (Sval < minS)
                {
                    minS = Sval;
                    bestDisp = (short)d;
                }
            }
            *mem.getMinLr(lrID, x) = (CostType)minL0;
            //}
            //else
            //{
             //   d = 0;
//
                     //   for (; d < D; d++)
                      //  {
                      //      int Sval = Sp[d];
                     //       if (Sval < minS)
                       //     {
                      //          minS = (CostType)Sval;
                       //         bestDisp = (short)d;
                         //   }
                     //   }
                    //}

            for (d = 0; d < D; d++)
            {
                if (Sp[d] * (100 - uniquenessRatio) < minS * 100 && std::abs(bestDisp - d) > 1)
                    break;
            }
            if (d < D)
                continue;
            d = bestDisp;
            int _x2 = x + minX1 - d - minD;
            if (mem.disp2cost[_x2] > minS)
            {
                mem.disp2cost[_x2] = (CostType)minS;
                mem.disp2ptr[_x2] = (DispType)(d + minD);
            }

            if (0 < d && d < D - 1)
            {

                int denom2 = max(Sp[d - 1] + Sp[d + 1] - 2 * Sp[d], 1);
                d = d * DISP_SCALE + ((Sp[d - 1] - Sp[d + 1]) * DISP_SCALE + denom2) / (denom2 * 2);
            }
            else
                d *= DISP_SCALE;
            disp1ptr[x + minX1] = (DispType)(d + minD * DISP_SCALE);
        }

        for (x = minX1; x < maxX1; x++)
        {

            int d1 = disp1ptr[x];
            if (d1 == INVALID_DISP_SCALED)
                continue;
            int _d = d1 >> DISP_SHIFT;
            int d_ = (d1 + DISP_SCALE - 1) >> DISP_SHIFT;
            int _x = x - _d, x_ = x - d_;
            if (0 <= _x && _x < width && mem.disp2ptr[_x] >= minD && std::abs(mem.disp2ptr[_x] - _d) > disp12MaxDiff &&
                0 <= x_ && x_ < width && mem.disp2ptr[x_] >= minD && std::abs(mem.disp2ptr[x_] - d_) > disp12MaxDiff)
                disp1ptr[x] = (DispType)INVALID_DISP_SCALED;
        }
        //}

        lrID = 1 - lrID; // now shift the cyclic buffers
    }
    //}
}


Mat buffer;
void compute(Mat& left, Mat& right, Mat& disp) //CV_OVERRIDE
{
    int minDisparity = 0;
    int numDisparities = 96;
    int SADWindowSize = 11;
    int P1 = 864;
    int P2 = 3456;
    int disp12MaxDiff = 0;
    int preFilterCap = 0;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = 0;

    //cout<<"compute"<<endl;
    disp.create_disp(Mat_buffer);

    computeDisparitySGBM__(left, right, disp);


}


Mat computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
{
    // Feel free to read OpenCV documentationand tweak these values.These work well
    int sad_window = 6;
    int num_disparities = sad_window * 16;
    int block_size = 11;

    Mat disp_left;
    Mat new_disp;

    //printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
    //clock_t start = clock();
    if (matcher_name == SGBM)
    {
        compute(img_left, img_right, disp_left);

        new_disp.cols = disp_left.cols;
        new_disp.rows = disp_left.rows;
        new_disp.step[0] = small_width * 4;
        new_disp.step[1] = 4;
        new_disp.flags = 1124024325;

        static unsigned char tempch[4 * small_width * small_height];
        new_disp.data = tempch;
        //new_disp.data = Mat_buffer;

        short temp;
        float temp2;

        for (int i = 0; i < new_disp.rows; i++)
        {
            for (int j = 0; j < new_disp.cols; j++)
            {
                temp = disp_left.at<short>(i, j);
                temp2 = static_cast<float>(temp) / 16;
                new_disp.at<float>(i, j) = temp2;
            }
        }

    }

    int x = 300, y = 1200;

    return new_disp;
}


void stereo_2_depth(unsigned char* img_left_input, unsigned char* img_right_input, unsigned char*k_left_input, unsigned char* t_left_input, unsigned char* t_right_input, unsigned char* depth_output)
{
	Mat img_left, img_right,k_left, t_left, t_right, depth;

    img_left.flags = 1124024320;
    img_left.dims = 2;
    img_left.step[0] = small_width;
    img_left.step[1] = 1;
    img_left.rows = small_height;
    img_left.cols = small_width;
    img_left.data = img_left_input;

    img_right.flags = 1124024320;
    img_right.dims = 2;
    img_right.step[0] = small_width;
    img_right.step[1] = 1;
    img_right.rows = small_height;
    img_right.cols = small_width;
    img_right.data = img_right_input;

    k_left.flags = 1124024325;
    k_left.dims = 2;
    k_left.step[0] = 12;
    k_left.step[1] = 4;
    k_left.rows = 3;
    k_left.cols = 3;
    k_left.data = k_left_input;

    t_left.flags = 1124024325;
    t_left.dims = 2;
    t_left.step[0] = 4;
    t_left.step[1] = 4;
    t_left.rows = 4;
    t_left.cols = 1;
    t_left.data = t_left_input;

    t_right.flags = 1124024325;
    t_right.dims = 2;
    t_right.step[0] = 4;
    t_right.step[1] = 4;
    t_right.rows = 4;
    t_right.cols = 1;
    t_right.data = t_right_input;

    depth.flags = 1124024325;
    depth.dims = 2;
    depth.step[0] = small_width * 4;
    depth.step[1] = 4;
    depth.rows = small_height;
    depth.cols = small_width;
    depth.data = depth_output;
    
    
    Mat disp, depth_map;
    
    disp = computeLeftDisparityMap(img_left, img_right, SGBM, 0);

    //depth.create_ones(Mat_buffer);
    float f = k_left.at<float>(0, 0);
    float b;
    b = t_right.at<float>(0, 0) - t_left.at<float>(0, 0);

    for (int i = 0; i < disp.rows; i++)
    {
        for (int j = 0; j < disp.cols; j++)
        {
            // Avoid instability and division by zero
            if (disp.at<float>(i, j) == 0.0 ||
                disp.at<float>(i, j) == -1.0)
                disp.at<float>(i, j) = 0.1;
            // Make empty depth map then fill with depth
            depth.at<float>(i, j) = f * b / disp.at<float>(i, j);
        }
    }
    //return depth;
}

void top_function(unsigned char* image_left_test_data, unsigned char* image_right_test_data, unsigned char* k_left_test_data, unsigned char* t_left_test_data, unsigned char* t_right_test_data, unsigned char* depth_map)

{
#pragma HLS INTERFACE mode=m_axi depth=1241*376 bundle=BUS_A port=image_left_test_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=1241*376 bundle=BUS_B port=image_right_test_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=36 bundle=BUS_C port=k_left_test_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=16 bundle=BUS_D port=t_left_test_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=16 bundle=BUS_E port=t_right_test_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=1241*376*4 port=depth_map offset=slave
#pragma HLS INTERFACE mode=s_axilite port=image_left_test_data
#pragma HLS INTERFACE mode=s_axilite port=image_right_test_data
#pragma HLS INTERFACE mode=s_axilite port=k_left_test_data
#pragma HLS INTERFACE mode=s_axilite port=t_left_test_data
#pragma HLS INTERFACE mode=s_axilite port=t_right_test_data
#pragma HLS INTERFACE mode=s_axilite port=depth_map
#pragma HLS INTERFACE mode = s_axilite port = return


	unsigned char*img_left_1, *img_right_1, *img_left_2, *img_right_2, *img_left_3, *img_right_3, *k_left, *t_left, *t_right;
    //Mat depth_1, depth_2, depth_3;
    Mat disp;

    static unsigned char left_buf[small_width * small_height];
    static unsigned char right_buf[small_width * small_height];
    static unsigned char k_left_buf[4 * 3 * 3];
    static unsigned char t_left_buf[4 * 4 * 1];
    static unsigned char t_right_buf[4 * 4 * 1];
    static unsigned char depth_buf[small_width * small_height * 4];

    for (int i = 0; i < 36; i++)
    {
#pragma HLS PIPELINE
    	k_left_buf[i] = k_left_test_data[i];
    }

    for (int i = 0; i < 16; i++)
    {
#pragma HLS PIPELINE
        t_left_buf[i] = t_left_test_data[i];
    }

    for (int i = 0; i < 16; i++)
    {
#pragma HLS PIPELINE
        t_right_buf[i] = t_right_test_data[i];
    }


    //first_img----------------------------------------------------------
    int counter_1 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 0; j < 414; j++)
        {
#pragma HLS PIPELINE
            left_buf[counter_1] = image_left_test_data[i + j];
            counter_1++;
        }
    }

    int counter_2 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 0; j < 414; j++)
        {
#pragma HLS PIPELINE
            right_buf[counter_2] = image_right_test_data[i + j];
            counter_2++;
        }
    }

    stereo_2_depth(left_buf, right_buf, k_left_buf, t_left_buf, t_right_buf, depth_buf);
    
        int counter_20 = 0;
    for (int i = 0; i < 4 * small_width * small_height; i += 4 * 414)
    {
        for (int j = 4 * 0; j < 4 * 306; j++)
        {
#pragma HLS PIPELINE
            depth_buf[counter_20] = depth_buf[i + j];
            counter_20++;
        }
    }

    int depth_counter_1 = 0;
    for (int i = 0; i < 4 * whole_width * whole_height; i += (1241 * 4))
    {
        for (int j = 0; j < 4 * 306; j++)
        {
#pragma HLS PIPELINE
            depth_map[i + j] = depth_buf[depth_counter_1];
            depth_counter_1++;
        }
    }
    //second_img----------------------------------------------------------
    int counter_3 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 197; j < 611; j++)
        {
#pragma HLS PIPELINE
            left_buf[counter_3] = image_left_test_data[i + j];
            counter_3++;
        }
    }

    int counter_4 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 197; j < 611; j++)
        {
#pragma HLS PIPELINE
            right_buf[counter_4] = image_right_test_data[i + j];
            counter_4++;
        }
    }


    stereo_2_depth(left_buf, right_buf, k_left_buf, t_left_buf, t_right_buf, depth_buf);


    int counter_11 = 0;
    for (int i = 0; i < 4 * small_width * small_height; i += 4 * 414)
    {
        for (int j = 4 * 108; j < 4 * 318; j++)
        {
#pragma HLS PIPELINE
            depth_buf[counter_11] = depth_buf[i + j];
            counter_11++;
        }
    }

    int depth_counter_2 = 0;
    for (int i = 0; i < 4 * whole_width * whole_height; i += (1241 * 4))
    {
        for (int j = 4 * 305; j < 4 * 515; j++)
        {
#pragma HLS PIPELINE
            depth_map[i + j] = depth_buf[depth_counter_2];
            depth_counter_2++;
        }
    }

    //third_img----------------------------------------------------------
    int counter_5 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 407; j < 821; j++)
        {
#pragma HLS PIPELINE
            left_buf[counter_5] = image_left_test_data[i + j];
            counter_5++;
        }
    }

    int counter_6 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 407; j < 821; j++)
        {
#pragma HLS PIPELINE
            right_buf[counter_6] = image_right_test_data[i + j];
            counter_6++;
        }
    }

    stereo_2_depth(left_buf, right_buf, k_left_buf, t_left_buf, t_right_buf, depth_buf);

    int counter_12 = 0;
    for (int i = 0; i < 4 * small_width * small_height; i += 4 * 414)
    {
        for (int j = 4 * 108; j < 4 * 318; j++)
        {
#pragma HLS PIPELINE
            depth_buf[counter_12] = depth_buf[i + j];
            counter_12++;
        }
    }

    int depth_counter_3 = 0;
    for (int i = 0; i < 4 * whole_width * whole_height; i += (1241 * 4))
    {
        for (int j = 4 * 515; j < 4 * 725; j++)
        {
#pragma HLS PIPELINE
            depth_map[i + j] = depth_buf[depth_counter_3];
            depth_counter_3++;
        }
    }

    //fourth_img----------------------------------------------------------//no
    int counter_7 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 617 ; j < 1031; j++)
        {
#pragma HLS PIPELINE
            left_buf[counter_7] = image_left_test_data[i + j];
            counter_7++;
        }
    }

    int counter_8 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 617; j < 1031; j++)
        {
#pragma HLS PIPELINE
            right_buf[counter_8] = image_right_test_data[i + j];
            counter_8++;
        }
    }

    stereo_2_depth(left_buf, right_buf, k_left_buf, t_left_buf, t_right_buf, depth_buf);

    int counter_13 = 0;
    for (int i = 0; i < 4 * small_width * small_height; i += 4 * 414)
    {
        for (int j = 4 * 108; j < 4 * 318; j++)
        {
#pragma HLS PIPELINE
            depth_buf[counter_13] = depth_buf[i + j];
            counter_13++;
        }
    }

    int depth_counter_4 = 0;
    for (int i = 0; i < 4 * whole_width * whole_height; i += (1241 * 4))
    {
        for (int j = 4 * 725; j < 4 * 935; j++)
        {
#pragma HLS PIPELINE
            depth_map[i + j] = depth_buf[depth_counter_4];
            depth_counter_4++;
        }
    }

    //fifth_img----------------------------------------------------------
    int counter_9 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 827; j < 1241; j++)
        {
#pragma HLS PIPELINE
            left_buf[counter_9] = image_left_test_data[i + j];
            counter_9++;
        }
    }

    int counter_10 = 0;
    for (int i = 0; i < whole_width * whole_height; i += 1241)
    {
        for (int j = 827; j < 1241; j++)
        {
#pragma HLS PIPELINE
            right_buf[counter_10] = image_right_test_data[i + j];
            counter_10++;
        }
    }

    stereo_2_depth(left_buf, right_buf, k_left_buf, t_left_buf, t_right_buf, depth_buf);

    int counter_14 = 0;
    for (int i = 0; i < 4 * small_width * small_height; i += 4 * 414)
    {
        for (int j = 4 * 108; j < 4 * 414; j++)
        {
#pragma HLS PIPELINE
            depth_buf[counter_14] = depth_buf[i + j];
            counter_14++;
        }
    }
    
    int depth_counter_5 = 0;
    for (int i = 0; i < 4 * whole_width * whole_height; i += (1241 * 4))
    {
        for (int j = 4 * 935; j < 4 * 1241; j++)
        {
#pragma HLS PIPELINE
            depth_map[i + j] = depth_buf[depth_counter_5];
            depth_counter_5++;
        }
    }

}





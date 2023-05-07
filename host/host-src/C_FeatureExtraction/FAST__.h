
//#include "BufferArea__.h"
#include "define.h"
#include "KeyPoint.h"

using namespace std;


void makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    //cout << "rowStride" << rowStride << endl;
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int(*offsets)[2] = patternSize == 16 ? offsets16 :
        patternSize == 12 ? offsets12 :
        patternSize == 8 ? offsets8 : 0;

    //CV_Assert(pixel && offsets);

    int k = 0;
    for (; k < patternSize; k++)
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for (; k < 25; k++)
        pixel[k] = pixel[k - patternSize];
}
/*
template<int patternSize>
int cornerScore(const uchar* ptr, const int pixel[], int threshold);

template<>
*/
int cornerScore(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K * 3 + 1;
    int k, v = ptr[0];
    short d[N];
    for (k = 0; k < N; k++)
        d[k] = (short)(v - ptr[pixel[k]]);


    {

        int a0 = threshold;
        for (k = 0; k < 16; k += 2)
        {
            int a = min((int)d[k + 1], (int)d[k + 2]);
            a = min(a, (int)d[k + 3]);
            if (a <= a0)
                continue;
            a = min(a, (int)d[k + 4]);
            a = min(a, (int)d[k + 5]);
            a = min(a, (int)d[k + 6]);
            a = min(a, (int)d[k + 7]);
            a = min(a, (int)d[k + 8]);
            a0 = max(a0, min(a, (int)d[k]));
            a0 = max(a0, min(a, (int)d[k + 9]));
        }

        int b0 = -a0;
        for (k = 0; k < 16; k += 2)
        {
            int b = max((int)d[k + 1], (int)d[k + 2]);
            b = max(b, (int)d[k + 3]);
            b = max(b, (int)d[k + 4]);
            b = max(b, (int)d[k + 5]);
            if (b >= b0)
                continue;
            b = max(b, (int)d[k + 6]);
            b = max(b, (int)d[k + 7]);
            b = max(b, (int)d[k + 8]);

            b0 = min(b0, max(b, (int)d[k]));
            b0 = min(b0, max(b, (int)d[k + 9]));
        }

        threshold = -b0 - 1;
    }


    return threshold;
}


/*
template<int patternSize>
void FAST_t(Mat& img, std::vector<KeyPoint>& keypoints, int threshold, bool nonmax_suppression)
{
    //Mat img = _img.getMat();
    //outmat1(img);
    const int K = patternSize / 2, N = patternSize + K + 1;
    int i, j, k, pixel[25];
    makeOffsets(pixel, (int)img.step[0], patternSize);

    //for (i = 0; i < 25; i++)
        //cout << "i: " << i << " " << pixel[i] << endl;

#if CV_SIMD128
    const int quarterPatternSize = patternSize / 4;
    v_uint8x16 delta = v_setall_u8(0x80), t = v_setall_u8((char)threshold), K16 = v_setall_u8((char)K);
#if CV_TRY_AVX2
    Ptr<opt_AVX2::FAST_t_patternSize16_AVX2> fast_t_impl_avx2;
    if (CV_CPU_HAS_SUPPORT_AVX2)
        fast_t_impl_avx2 = opt_AVX2::FAST_t_patternSize16_AVX2::getImpl(img.cols, threshold, nonmax_suppression, pixel);
#endif

#endif
    
    keypoints.clear();

    threshold = min(max(threshold, 0), 255);
    //cout << "keypoints size: " << threshold << endl;

    uchar threshold_tab[512];
    for (i = -255; i <= 255; i++)
        threshold_tab[i + 255] = (uchar)(i < -threshold ? 1 : i > threshold ? 2 : 0);

   // for (i = -255; i <= 255; i++)
   //     cout << "i: " << i << " " << static_cast<int>(threshold_tab[i + 255]) << endl;

    uchar* buf[3] = { 0 };
    int* cpbuf[3] = { 0 };
    BufferArea__ area;
    for (unsigned idx = 0; idx < 3; ++idx)
    {
        area.allocate(buf[idx], img.cols);
        area.allocate(cpbuf[idx], img.cols + 1);
    }
    area.commit();

    for (unsigned idx = 0; idx < 3; ++idx)
    {
        memset(buf[idx], 0, img.cols);
    }
   // outmat1(img);
    for (i = 3; i < img.rows - 2; i++)
    {
        const uchar* ptr = img.ptr<uchar>(i) + 3;
        uchar* curr = buf[(i - 3) % 3];
        int* cornerpos = cpbuf[(i - 3) % 3] + 1; // cornerpos[-1] is used to store a value
        memset(curr, 0, img.cols);
        int ncorners = 0;

        if (i < img.rows - 3)
        {
            j = 3;

            for (; j < img.cols - 3; j++, ptr++)
            {
                int v = ptr[0];
                const uchar* tab = &threshold_tab[0] - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if (d == 0)
                    continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if (d == 0)
                    continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if (d & 1)
                {
                    int vt = v - threshold, count = 0;

                    for (k = 0; k < N; k++)
                    {
                        int x = ptr[pixel[k]];
                        if (x < vt)
                        {
                            if (++count > K)
                            {
                                cornerpos[ncorners++] = j;
                                if (nonmax_suppression)
                                    curr[j] = (uchar)cornerScore<patternSize>(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }

                if (d & 2)
                {
                    int vt = v + threshold, count = 0;

                    for (k = 0; k < N; k++)
                    {
                        int x = ptr[pixel[k]];
                        if (x > vt)
                        {
                            if (++count > K)
                            {
                                cornerpos[ncorners++] = j;
                                if (nonmax_suppression)
                                    curr[j] = (uchar)cornerScore<patternSize>(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }
            }
        }

        cornerpos[-1] = ncorners;

        if (i == 3)
            continue;

        const uchar* prev = buf[(i - 4 + 3) % 3];
        const uchar* pprev = buf[(i - 5 + 3) % 3];
        cornerpos = cpbuf[(i - 4 + 3) % 3] + 1; // cornerpos[-1] is used to store a value
        ncorners = cornerpos[-1];

        for (k = 0; k < ncorners; k++)
        {
            j = cornerpos[k];
            int score = prev[j];
            if (!nonmax_suppression ||
                (score > prev[j + 1] && score > prev[j - 1] &&
                    score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
                    score > curr[j - 1] && score > curr[j] && score > curr[j + 1]))
            {
                keypoints.push_back(KeyPoint((float)j, (float)(i - 1), 7.f, -1, (float)score));
            }
        }
    }
}
*/




void FAST__(Mat& img, KeyPoint* keypoints, int threshold, bool nonmax_suppression , int& kpSize)
{

	const int patternSize = 16;
	const int K = patternSize / 2, N = patternSize + K + 1;
	int i, j, k, pixel[25];
#pragma HLS ARRAY_PARTITION variable=pixel complete dim=1

	makeOffsets(pixel, (int)img.step[0], patternSize);

	int KPcount = 0;

	    threshold = min(max(threshold, 0), 255);
	    //cout << "keypoints size: " << threshold << endl;

	    uchar threshold_tab[512];
	    for (i = -255; i <= 255; i++)
	        threshold_tab[i + 255] = (uchar)(i < -threshold ? 1 : i > threshold ? 2 : 0);

	    //for (i = -255; i <= 255; i++)
	    //    cout << "i: " << i << " " << static_cast<int>(threshold_tab[i + 255]) << endl;

	    //uchar* buf[3] = { 0 };
	    //int* cpbuf[3] = { 0 };

	    int cpstore[3];
	    /*BufferArea__ area;
	    for (unsigned idx = 0; idx < 3; ++idx)
	    {
	        area.allocate(buf[idx], img.cols);
	        area.allocate(cpbuf[idx], img.cols + 1);
	    }
	    area.commit();

	    for (unsigned idx = 0; idx < 3; ++idx)
	    {
	        memset(buf[idx], 0, img.cols);
	    }*/


	    static uchar currbuf0[1241];
	    static uchar currbuf1[1241];
	    static uchar currbuf2[1241];
	     //   buf[0] = currbuf0;
	      //  buf[1] = currbuf1;
	      //  buf[2] = currbuf2;
	        for (int idxc = 0; idxc < 1241; idxc++)
	        {
#pragma HLS PIPELINE
	        	currbuf0[idxc] = 0;
	        	currbuf1[idxc] = 0;
	        	currbuf2[idxc] = 0;

	        }

	        static int cornerposbuf0[1242];
	        static int cornerposbuf1[1242];
	        static int cornerposbuf2[1242];
	       // cpbuf[0] = cornerposbuf0;
	       // cpbuf[1] = cornerposbuf1;
	       // cpbuf[2] = cornerposbuf2;










	    for (i = 3; i < img.rows - 2; i++)
	    {
	        const uchar* ptr = img.ptr(i) + 3;
	        uchar* curr;//= buf[(i - 3) % 3];
	        int* cornerpos;// = cpbuf[(i - 3) % 3] + 1; // cornerpos[-1] is used to store a value
	        //memset(curr, 0, img.cols);


	        		if ((i - 3) % 3 == 0)
	                    curr = currbuf0;
	                else if ((i - 3) % 3 == 1)
	                    curr = currbuf1;
	                else
	                    curr = currbuf2;

	                if ((i - 3) % 3 == 0)
	                    cornerpos = cornerposbuf0 + 1;
	                else if ((i - 3) % 3 == 1)
	                    cornerpos = cornerposbuf1 + 1;
	                else
	                    cornerpos = cornerposbuf2 + 1;



	        for (int idxc = 0; idxc < 1241; idxc++)
	        {
#pragma HLS PIPELINE
	              curr[idxc] = 0;
	        }


	        int ncorners = 0;

	        if (i < img.rows - 3)
	        {
	            j = 3;

	            for (; j < img.cols - 3; j++, ptr++)
	            {
#pragma HLS loop_tripcount max=1238
#pragma HLS PIPELINE off

	                int v = ptr[0];
	                const uchar* tab = &threshold_tab[0] - v + 255;
	                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

	                if (d == 0)
	                    continue;

	                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
	                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
	                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

	                if (d == 0)
	                    continue;

	                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
	                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
	                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
	                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

	                if (d & 1)
	                {
	                    int vt = v - threshold, count = 0;

	                    for (k = 0; k < N; k++)
	                    {
#pragma HLS PIPELINE
	                        int x = ptr[pixel[k]];
	                        if (x < vt)
	                        {
	                            if (++count > K)
	                            {
	                                cornerpos[ncorners++] = j;
	                                if (nonmax_suppression)
	                                {

	                                   // curr[j] = (uchar)cornerScore(ptr, pixel, threshold);
	                                    const int Kc = 8, Nc = Kc * 3 + 1;
										int kc, vc = ptr[0];
										short dc[Nc];
									#pragma HLS ARRAY_PARTITION variable=dc type=complete dim=1
										for (kc = 0; kc < Nc; kc++)
											dc[kc] = (short)(vc - ptr[pixel[kc]]);


										//{

											int a0 = threshold;
											for (kc = 0; kc < 16; kc += 2)
											{
									#pragma HLS UNROLL
//#pragma HLS PIPELINE
												int a = min((int)dc[kc + 1], (int)dc[kc + 2]);
												a = min(a, (int)dc[kc + 3]);
												if (a <= a0)
													continue;
												a = min(a, (int)dc[kc + 4]);
												a = min(a, (int)dc[kc + 5]);
												a = min(a, (int)dc[kc + 6]);
												a = min(a, (int)dc[kc + 7]);
												a = min(a, (int)dc[kc + 8]);
												a0 = max(a0, min(a, (int)dc[kc]));
												a0 = max(a0, min(a, (int)dc[kc + 9]));
											}

											int b0 = -a0;
											for (kc = 0; kc < 16; kc += 2)
											{
									#pragma HLS UNROLL
//#pragma HLS PIPELINE
												int b = max((int)dc[kc + 1], (int)dc[kc + 2]);
												b = max(b, (int)dc[kc + 3]);
												b = max(b, (int)dc[kc + 4]);
												b = max(b, (int)dc[kc + 5]);
												if (b >= b0)
													continue;
												b = max(b, (int)dc[kc + 6]);
												b = max(b, (int)dc[kc + 7]);
												b = max(b, (int)dc[kc + 8]);

												b0 = min(b0, max(b, (int)dc[kc]));
												b0 = min(b0, max(b, (int)dc[kc + 9]));
											}

											curr[j] = (uchar)(-b0 - 1);
										//}


										//return threshold;
	                                }
	                                break;
	                            }
	                        }
	                        else
	                            count = 0;
	                    }
	                }

	                if (d & 2)
	                {

	                    int vt = v + threshold, count = 0;

	                    for (k = 0; k < N; k++)
	                    {
#pragma HLS PIPELINE
	                        int x = ptr[pixel[k]];
	                        if (x > vt)
	                        {
	                            if (++count > K)
	                            {
	                                cornerpos[ncorners++] = j;
	                                if (nonmax_suppression)
	                                {
	                                    //curr[j] = (uchar)cornerScore(ptr, pixel, threshold);
	                                    const int Kc = 8, Nc = Kc * 3 + 1;
										int kc, vc = ptr[0];
										short dc[Nc];
									#pragma HLS ARRAY_PARTITION variable=dc type=complete dim=1
										for (kc = 0; kc < Nc; kc++)
											dc[kc] = (short)(vc - ptr[pixel[kc]]);


										//{

											int a0 = threshold;
											for (kc = 0; kc < 16; kc += 2)
											{
									#pragma HLS UNROLL
//#pragma HLS PIPELINE
												int a = min((int)dc[kc + 1], (int)dc[kc + 2]);
												a = min(a, (int)dc[kc + 3]);
												if (a <= a0)
													continue;
												a = min(a, (int)dc[kc + 4]);
												a = min(a, (int)dc[kc + 5]);
												a = min(a, (int)dc[kc + 6]);
												a = min(a, (int)dc[kc + 7]);
												a = min(a, (int)dc[kc + 8]);
												a0 = max(a0, min(a, (int)dc[kc]));
												a0 = max(a0, min(a, (int)dc[kc + 9]));
											}

											int b0 = -a0;
											for (kc = 0; kc < 16; kc += 2)
											{
									#pragma HLS UNROLL
//#pragma HLS PIPELINE
												int b = max((int)dc[kc + 1], (int)dc[kc + 2]);
												b = max(b, (int)dc[kc + 3]);
												b = max(b, (int)dc[kc + 4]);
												b = max(b, (int)dc[kc + 5]);
												if (b >= b0)
													continue;
												b = max(b, (int)dc[kc + 6]);
												b = max(b, (int)dc[kc + 7]);
												b = max(b, (int)dc[kc + 8]);

												b0 = min(b0, max(b, (int)dc[kc]));
												b0 = min(b0, max(b, (int)dc[kc + 9]));
											}

											curr[j] = (uchar)(-b0 - 1);
										//}


										//return threshold;
	                                }
	                                break;
	                            }
	                        }
	                        else
	                            count = 0;
	                    }
	                }
	            }
	        }

	        //cornerpos[-1] = ncorners;
	        cpstore[(i - 3) % 3] = ncorners;

	        if (i == 3)
	            continue;

	        const uchar* prev;// = buf[(i - 4 + 3) % 3];
	        const uchar* pprev;// = buf[(i - 5 + 3) % 3];
	       // cornerpos = cpbuf[(i - 4 + 3) % 3] + 1; // cornerpos[-1] is used to store a value
	        //ncorners = cornerpos[-1];
	        ncorners = cpstore[(i - 4 + 3) % 3];


	        		if ((i - 4 + 3) % 3 == 0)
	                    prev = currbuf0;
	                else if ((i - 4 + 3) % 3 == 1)
	                    prev = currbuf1;
	                else
	                    prev = currbuf2;

	                if ((i - 5 + 3) % 3 == 0)
	                    pprev = currbuf0;
	                else if ((i - 5 + 3) % 3 == 1)
	                    pprev = currbuf1;
	                else
	                    pprev = currbuf2;

	                if ((i - 4 + 3) % 3 == 0)
	                    cornerpos = cornerposbuf0 + 1;
	                else if ((i - 4 + 3) % 3 == 1)
	                    cornerpos = cornerposbuf1 + 1;
	                else
	                    cornerpos = cornerposbuf2 + 1;




	        for (k = 0; k < ncorners; k++)
	        {
#pragma HLS PIPELINE
	            j = cornerpos[k];
	            int score = prev[j];
	            if (!nonmax_suppression ||
	                (score > prev[j + 1] && score > prev[j - 1] &&
	                    score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
	                    score > curr[j - 1] && score > curr[j] && score > curr[j + 1]))
	            {
	               // keypoints.push_back(KeyPoint((float)j, (float)(i - 1), 7.f, -1, (float)score));
	                keypoints[KPcount] = KeyPoint((float)j, (float)(i - 1), 7.f, -1, (float)score);
	                KPcount++;
	            }
	        }
	    }

	    kpSize = KPcount;






}

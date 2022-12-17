#include<opencv2/opencv.hpp>
#include<opencv2/core/core_c.h>
#include<opencv2/calib3d/calib3d_c.h>
#include"epnp.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

using namespace std;
using namespace cv;

#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0




void estimate_motion(vector<vector<DMatch>> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat k, Mat depth1, int max_depth,
	Mat& rmat, Mat& tvec, Mat& image1_points, Mat& image2_points, int maxIter, float error, float confidence)
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
	Mat rvec, rvec_test, rmat_test, tvec_test;
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
		__solvePnPRansac__(object_points, image2_points, k, distCoef, rvec_test, tvec_test,
			false, maxIter, error, confidence, SOLVEPNP_ITERATIVE);

		Mat inlier;
		cv::solvePnPRansac(object_points, image2_points, k, distCoef, rvec, tvec,
			false, maxIter, error, confidence, inlier, SOLVEPNP_ITERATIVE);

		rmat = Mat::eye(3, 3, CV_32F);
		rmat_test = Mat::eye(3, 3, CV_32F);
		Rodrigues(rvec, rmat);
		Rodrigues(rvec_test, rmat_test);

		if (countNonZero(rvec != rvec_test)) {
			printf("================ not equal =================\n");
			cout << "rvec\n" << rvec << endl;
			cout << "rvec test\n" << rvec_test << endl;
		}
		if (countNonZero(tvec != tvec_test)) {
			printf("================ not equal =================\n");
			cout << "tvec\n" << tvec << endl;
			cout << "tvec test\n" << tvec_test << endl;
		}


	}
}


struct SVDD
{
	Mat u;
	Mat vt;
	Mat w;
} ;


void _JacobiSVDImpl_(double* At, size_t astep, double* _W, double* Vt, size_t vstep,
	int m, int n, int n1, double minval, double eps)
{
	AutoBuffer<double> Wbuf(n);
	double* W = Wbuf.data();
	int i, j, k, iter, max_iter = max(m, 30);
	double c, s;
	double sd;
	astep /= sizeof(At[0]);
	vstep /= sizeof(Vt[0]);

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			double t = At[i * astep + k];
			sd += (double)t * t;
		}
		W[i] = sd;

		for (k = 0; k < n; k++)
				Vt[i * vstep + k] = 0;
			Vt[i * vstep + i] = 1;
	}

	for (iter = 0; iter < max_iter; iter++)
	{
		bool changed = false;

		for (i = 0; i < n - 1; i++)
			for (j = i + 1; j < n; j++)
			{
				double* Ai = At + i * astep, * Aj = At + j * astep;
				double a = W[i], p = 0, b = W[j];

				for (k = 0; k < m; k++)
					p += (double)Ai[k] * Aj[k];

				if (std::abs(p) <= eps * std::sqrt((double)a * b))
					continue;

				p *= 2;
				double beta = a - b, gamma = hypot((double)p, beta);
				if (beta < 0)
				{
					double delta = (gamma - beta) * 0.5;
					s = (double)std::sqrt(delta / gamma);
					c = (double)(p / (gamma * s * 2));
				}
				else
				{
					c = (double)std::sqrt((gamma + beta) / (gamma * 2));
					s = (double)(p / (gamma * c * 2));
				}

				a = b = 0;
				for (k = 0; k < m; k++)
				{
					double t0 = c * Ai[k] + s * Aj[k];
					double t1 = -s * Ai[k] + c * Aj[k];
					Ai[k] = t0; Aj[k] = t1;

					a += (double)t0 * t0; b += (double)t1 * t1;
				}
				W[i] = a; W[j] = b;

				changed = true;

				double* Vi = Vt + i * vstep, * Vj = Vt + j * vstep;
				k = 0;

				for (; k < n; k++)
				{
					double t0 = c * Vi[k] + s * Vj[k];
					double t1 = -s * Vi[k] + c * Vj[k];
					Vi[k] = t0; Vj[k] = t1;
				}
			}
		if (!changed)
			break;
	}

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			double t = At[i * astep + k];
			sd += (double)t * t;
		}
		W[i] = std::sqrt(sd);
	}

	for (i = 0; i < n - 1; i++)
	{
		j = i;
		for (k = i + 1; k < n; k++)
		{
			if (W[j] < W[k])
				j = k;
		}
		if (i != j)
		{
			std::swap(W[i], W[j]);
			if (Vt)
			{
				for (k = 0; k < m; k++)
					std::swap(At[i * astep + k], At[j * astep + k]);

				for (k = 0; k < n; k++)
					std::swap(Vt[i * vstep + k], Vt[j * vstep + k]);
			}
		}
	}

	for (i = 0; i < n; i++)
		_W[i] = (double)W[i];

	if (!Vt)
		return;

	RNG rng(0x12345678);
	for (i = 0; i < n1; i++)
	{
		sd = i < n ? W[i] : 0;

		for (int ii = 0; ii < 100 && sd <= minval; ii++)
		{
			// if we got a zero singular value, then in order to get the corresponding left singular vector
			// we generate a random vector, project it to the previously computed left singular vectors,
			// subtract the projection and normalize the difference.
			const double val0 = (double)(1. / m);
			for (k = 0; k < m; k++)
			{
				double val = (rng.next() & 256) != 0 ? val0 : -val0;
				At[i * astep + k] = val;
			}
			for (iter = 0; iter < 2; iter++)
			{
				for (j = 0; j < i; j++)
				{
					sd = 0;
					for (k = 0; k < m; k++)
						sd += At[i * astep + k] * At[j * astep + k];
					double asum = 0;
					for (k = 0; k < m; k++)
					{
						double t = (double)(At[i * astep + k] - sd * At[j * astep + k]);
						At[i * astep + k] = t;
						asum += std::abs(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < m; k++)
						At[i * astep + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < m; k++)
			{
				double t = At[i * astep + k];
				sd += (double)t * t;
			}
			sd = std::sqrt(sd);
		}

		s = (double)(sd > minval ? 1 / sd : 0.);
		for (k = 0; k < m; k++)
			At[i * astep + k] *= s;
	}
}

/*
void _SVD64f(double* At, size_t astep, double* W, double* U, size_t ustep, double* Vt, size_t vstep, int m, int n, int n1)
{
	// CALL_HAL(SVD64f, cv_hal_SVD64f, At, astep, W, U, ustep, Vt, vstep, m, n, decodeSVDParameters(U, Vt, m, n, n1))
	_JacobiSVDImpl_(At, astep, W, Vt, vstep, m, n, !Vt ? 0 : n1 < 0 ? n : n1, DBL_MIN, DBL_EPSILON * 10);
}*/


static void JacobiSVD(double* At, size_t astep, double* _W, double* Vt, size_t vstep, int m, int n, int n11 = -1)
{
	double minval = DBL_MIN;
	double eps = DBL_EPSILON * 10;
	int n1;
	if (!Vt)
	{
		n1 = 0;
	}
	else {
		if (n11 < 0)
			n1 = n;
		else
			n1 = n11;
	}

	//VBLAS<double> vblas;
	AutoBuffer<double> Wbuf(n);
	double* W = Wbuf.data();
	int i, j, k, iter, max_iter = max(m, 30);
	double c, s;
	double sd;
	astep /= sizeof(At[0]);
	vstep /= sizeof(Vt[0]);

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			double t = At[i * astep + k];
			sd += (double)t * t;
		}
		W[i] = sd;

		for (k = 0; k < n; k++)
			Vt[i * vstep + k] = 0;
		Vt[i * vstep + i] = 1;
	}

	for (iter = 0; iter < max_iter; iter++)
	{
		bool changed = false;

		for (i = 0; i < n - 1; i++)
			for (j = i + 1; j < n; j++)
			{
				double* Ai = At + i * astep, * Aj = At + j * astep;
				double a = W[i], p = 0, b = W[j];

				for (k = 0; k < m; k++)
					p += (double)Ai[k] * Aj[k];

				if (std::abs(p) <= eps * std::sqrt((double)a * b))
					continue;

				p *= 2;
				double beta = a - b, gamma = hypot((double)p, beta);
				if (beta < 0)
				{
					double delta = (gamma - beta) * 0.5;
					s = (double)std::sqrt(delta / gamma);
					c = (double)(p / (gamma * s * 2));
				}
				else
				{
					c = (double)std::sqrt((gamma + beta) / (gamma * 2));
					s = (double)(p / (gamma * c * 2));
				}

				a = b = 0;
				for (k = 0; k < m; k++)
				{
					double t0 = c * Ai[k] + s * Aj[k];
					double t1 = -s * Ai[k] + c * Aj[k];
					Ai[k] = t0; Aj[k] = t1;

					a += (double)t0 * t0; b += (double)t1 * t1;
				}
				W[i] = a; W[j] = b;

				changed = true;

				double* Vi = Vt + i * vstep, * Vj = Vt + j * vstep;
				k = 0;//vblas.givens(Vi, Vj, n, c, s);

				for (; k < n; k++)
				{
					double t0 = c * Vi[k] + s * Vj[k];
					double t1 = -s * Vi[k] + c * Vj[k];
					Vi[k] = t0; Vj[k] = t1;
				}
			}
		if (!changed)
			break;
	}

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			double t = At[i * astep + k];
			sd += (double)t * t;
		}
		W[i] = std::sqrt(sd);
	}

	for (i = 0; i < n - 1; i++)
	{
		j = i;
		for (k = i + 1; k < n; k++)
		{
			if (W[j] < W[k])
				j = k;
		}
		if (i != j)
		{
			std::swap(W[i], W[j]);
			if (Vt)
			{
				for (k = 0; k < m; k++)
					std::swap(At[i * astep + k], At[j * astep + k]);

				for (k = 0; k < n; k++)
					std::swap(Vt[i * vstep + k], Vt[j * vstep + k]);
			}
		}
	}

	for (i = 0; i < n; i++)
		_W[i] = (double)W[i];

	if (!Vt)
		return;

	RNG rng(0x12345678);
	for (i = 0; i < n1; i++)
	{
		sd = i < n ? W[i] : 0;

		for (int ii = 0; ii < 100 && sd <= minval; ii++)
		{
			// if we got a zero singular value, then in order to get the corresponding left singular vector
			// we generate a random vector, project it to the previously computed left singular vectors,
			// subtract the projection and normalize the difference.
			const double val0 = (double)(1. / m);
			for (k = 0; k < m; k++)
			{
				double val = (rng.next() & 256) != 0 ? val0 : -val0;
				At[i * astep + k] = val;
			}
			for (iter = 0; iter < 2; iter++)
			{
				for (j = 0; j < i; j++)
				{
					sd = 0;
					for (k = 0; k < m; k++)
						sd += At[i * astep + k] * At[j * astep + k];
					double asum = 0;
					for (k = 0; k < m; k++)
					{
						double t = (double)(At[i * astep + k] - sd * At[j * astep + k]);
						At[i * astep + k] = t;
						asum += std::abs(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < m; k++)
						At[i * astep + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < m; k++)
			{
				double t = At[i * astep + k];
				sd += (double)t * t;
			}
			sd = std::sqrt(sd);
		}

		s = (double)(sd > minval ? 1 / sd : 0.);
		for (k = 0; k < m; k++)
			At[i * astep + k] *= s;
	}
}


void _SVDcompute(InputArray _aarr, OutputArray _w,
	OutputArray _u, OutputArray _vt, int flags)
{
	Mat src = _aarr.getMat();
	int m = src.rows, n = src.cols;
	int type = src.type();


	int urows = n;
	size_t esz = src.elemSize(), astep = alignSize(m * esz, 16), vstep = alignSize(n * esz, 16);
	AutoBuffer<uchar> _buf(urows * astep + n * vstep + n * esz + 32);
	uchar* buf = alignPtr(_buf.data(), 16);
	Mat temp_a(n, m, type, buf, astep);
	Mat temp_w(n, 1, type, buf + urows * astep);
	Mat temp_u(urows, m, type, buf, astep), temp_v;

	temp_v = Mat(n, n, type, alignPtr(buf + urows * astep + n * esz, 16), vstep);

	transpose(src, temp_a);

	JacobiSVD(temp_a.ptr<double>(), temp_u.step, temp_w.ptr<double>(),
		temp_v.ptr<double>(), temp_v.step, m, n, urows);

	temp_w.copyTo(_w);
	temp_v.copyTo(_vt);
	transpose(temp_u, _u);
}

void __cvSVD(CvArr* aarr, CvArr* warr, CvArr* uarr, CvArr* varr, int flags)
{
	Mat a = cvarrToMat(aarr), w = cvarrToMat(warr), u, v;
	int m = a.rows, n = a.cols, type = a.type(), mn = max(m, n), nm = min(m, n);


	SVDD svd;

	svd.w = w;

	if (uarr)
	{
		u = cvarrToMat(uarr);
		svd.u = u;
	}

	v = cvarrToMat(varr);
	svd.vt = v;

	/*svd(a, ((flags & CV_SVD_MODIFY_A) ? SVD::MODIFY_A : 0) |
		((!svd.u.data && !svd.vt.data) ? SVD::NO_UV : 0) |
		((m != n && (svd.u.size() == Size(mn, mn) ||
			svd.vt.size() == Size(mn, mn))) ? SVD::FULL_UV : 0));*/
	_SVDcompute(a, svd.w, svd.u, svd.vt, ((flags & CV_SVD_MODIFY_A) ? SVD::MODIFY_A : 0) |
		((!svd.u.data && !svd.vt.data) ? SVD::NO_UV : 0) |
		((m != n && (svd.u.size() == Size(mn, mn) ||
			svd.vt.size() == Size(mn, mn))) ? SVD::FULL_UV : 0));

	if (flags & CV_SVD_U_T) 
		transpose(svd.u, u);

}

//https://www.cnblogs.com/willhua/p/12521581.html



int cvRodrigues2(const CvMat* src, CvMat* dst, CvMat* jacobian)
{
	double J[27] = { 0 };
	CvMat matJ = cvMat(3, 9, CV_64F, J);

	int elem_size = CV_ELEM_SIZE(CV_MAT_DEPTH(src->type)); // = 8

	if (src->cols == 1 || src->rows == 1)
	{
		int step = src->rows > 1 ? src->step / elem_size : 1;

		Point3d r;

		r.x = src->data.db[0];
		r.y = src->data.db[step];
		r.z = src->data.db[step * 2];

		double theta = sqrt((double)r.x * r.x + (double)r.y * r.y + (double)r.z * r.z);

		double c = cos(theta);
		double s = sin(theta);
		double c1 = 1. - c;
		double itheta = theta ? 1. / theta : 0.;

		r *= itheta;

		Matx33d rrt(r.x * r.x, r.x * r.y, r.x * r.z,
					r.x * r.y, r.y * r.y, r.y * r.z,
					r.x * r.z, r.y * r.z, r.z * r.z);	//r cross r
		Matx33d r_x(0, -r.z, r.y,
					r.z, 0, -r.x,
					-r.y, r.x, 0);

		Matx33d R = c * Matx33d::eye() + c1 * rrt + s * r_x;

		Mat(R).convertTo(cvarrToMat(dst), dst->type);

		if (jacobian)
		{
			const double I[] = { 1, 0, 0, 
								0, 1, 0,
								0, 0, 1 };
			double drrt[] = { r.x + r.x, r.y, r.z, r.y, 0, 0, r.z, 0, 0,
							  0, r.x, 0, r.x, r.y + r.y, r.z, 0, r.z, 0,
							  0, 0, r.x, 0, 0, r.y, r.x, r.y, r.z + r.z };
			double d_r_x_[] = { 0, 0, 0, 0, 0, -1, 0, 1, 0,
								0, 0, 1, 0, 0, 0, -1, 0, 0,
								0, -1, 0, 1, 0, 0, 0, 0, 0 };
			for (int i = 0; i < 3; i++)
			{
				double ri = i == 0 ? r.x : i == 1 ? r.y : r.z;
				double a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
				double a3 = (c - s * itheta) * ri, a4 = s * itheta;
				for (int k = 0; k < 9; k++)
					J[i * 9 + k] = a0 * I[k] + a1 * rrt.val[k] + a2 * drrt[i * 9 + k] +
					a3 * r_x.val[k] + a4 * d_r_x_[i * 9 + k];
			}
		}
	}
	else if (src->cols == 3 && src->rows == 3)
	{
		Matx33d U, Vt;
		Vec3d W;
		double theta, s, c;
		int step = dst->rows > 1 ? dst->step / elem_size : 1;

		Matx33d R = cvarrToMat(src);

		SVD::compute(R, W, U, Vt);						// 1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		R = U * Vt;

		Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

		s = std::sqrt((r.x * r.x + r.y * r.y + r.z * r.z) * 0.25);
		c = (R(0, 0) + R(1, 1) + R(2, 2) - 1) * 0.5;
		c = c > 1. ? 1. : c < -1. ? -1. : c;
		theta = acos(c);

		double vth = 1 / (2 * s);

		if (jacobian)
		{
			double t, dtheta_dtr = -1. / s;
			double dvth_dtheta = -vth * c / s;
			double d1 = 0.5 * dvth_dtheta * dtheta_dtr;
			double d2 = 0.5 * dtheta_dtr;
			double dvardR[5 * 9] =
			{
				0, 0, 0, 0, 0, 1, 0, -1, 0,
				0, 0, -1, 0, 0, 0, 1, 0, 0,
				0, 1, 0, -1, 0, 0, 0, 0, 0,
				d1, 0, 0, 0, d1, 0, 0, 0, d1,
				d2, 0, 0, 0, d2, 0, 0, 0, d2
			};
			double dvar2dvar[] =
			{
				vth, 0, 0, r.x, 0,
				0, vth, 0, r.y, 0,
				0, 0, vth, r.z, 0,
				0, 0, 0, 0, 1
			};
			double domegadvar2[] =
			{
				theta, 0, 0, r.x * vth,
				0, theta, 0, r.y * vth,
				0, 0, theta, r.z * vth
			};

			CvMat _dvardR = cvMat(5, 9, CV_64FC1, dvardR);
			CvMat _dvar2dvar = cvMat(4, 5, CV_64FC1, dvar2dvar);
			CvMat _domegadvar2 = cvMat(3, 4, CV_64FC1, domegadvar2);
			double t0[3 * 5];
			CvMat _t0 = cvMat(3, 5, CV_64FC1, t0);

			cvMatMul(&_domegadvar2, &_dvar2dvar, &_t0);
			cvMatMul(&_t0, &_dvardR, &matJ);

			// transpose every row of matJ (treat the rows as 3x3 matrices)
			CV_SWAP(J[1], J[3], t); CV_SWAP(J[2], J[6], t); CV_SWAP(J[5], J[7], t);
			CV_SWAP(J[10], J[12], t); CV_SWAP(J[11], J[15], t); CV_SWAP(J[14], J[16], t);
			CV_SWAP(J[19], J[21], t); CV_SWAP(J[20], J[24], t); CV_SWAP(J[23], J[25], t);
		}

		vth *= theta;
		r *= vth;

		dst->data.db[0] = r.x;
		dst->data.db[step] = r.y;
		dst->data.db[step * 2] = r.z;
	}

	if (jacobian)
	{
		cvCopy(&matJ, jacobian);
	}

	return 1;
}







void cvProjectPoints2(const CvMat* objectPoints,
	const CvMat* r_vec,
	const CvMat* t_vec,
	const CvMat* A,
	const CvMat* distCoeffs,
	CvMat* imagePoints, CvMat* dpdr,
	CvMat* dpdt)
{
	Ptr<CvMat> matM, _m;
	Ptr<CvMat> _dpdr, _dpdt;

	int i, j, count;
	int calc_derivatives;
	const CvPoint3D64f* M;
	CvPoint2D64f* m;
	double r[3], R[9], dRdr[27], t[3], a[9], k[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}, fx, fy, cx, cy;
	Matx33d matTilt = Matx33d::eye();
	CvMat _r, matR = cvMat(3, 3, CV_64F, R);
	CvMat _t, _a = cvMat(3, 3, CV_64F, a);
	CvMat _dRdr = cvMat(3, 9, CV_64F, dRdr);
	double* dpdr_p = 0, * dpdt_p = 0;
	int dpdr_step = 0, dpdt_step = 0;

	int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type);//3

	count = total / 3;

	matM.reset(cvCreateMat(objectPoints->rows, objectPoints->cols, CV_64FC3));
	cvConvert(objectPoints, matM);

	_m.reset(cvCreateMat(imagePoints->rows, imagePoints->cols, CV_64FC2));
	cvConvert(imagePoints, _m);

	M = (CvPoint3D64f*)matM->data.db;
	m = (CvPoint2D64f*)_m->data.db;

	_r = cvMat(r_vec->rows, r_vec->cols, CV_64FC1, r);
	cvConvert(r_vec, &_r); 
	//Rodrigues(_r, matR, _dRdr);
	cvRodrigues2(&_r, &matR, &_dRdr);

	_t = cvMat(t_vec->rows, t_vec->cols, CV_64FC1, t);
	cvConvert(t_vec, &_t);


	cvConvert(A, &_a);
	fx = a[0]; fy = a[4];
	cx = a[2]; cy = a[5];


	if (dpdr)
	{
		_dpdr.reset(cvCloneMat(dpdr));
		dpdr_p = _dpdr->data.db;
		dpdr_step = _dpdr->step / sizeof(dpdr_p[0]);
	}

	if (dpdt)
	{
		_dpdt.reset(cvCloneMat(dpdt));

		dpdt_p = _dpdt->data.db;
		dpdt_step = _dpdt->step / sizeof(dpdt_p[0]);
	}

	calc_derivatives = dpdr || dpdt;

	for (i = 0; i < count; i++)
	{

		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
		double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
		double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2;
		double xd, yd, xd0, yd0, invProj;
		Vec3d vecTilt;
		Vec3d dVecTilt;
		Matx22d dMatTilt;
		Vec2d dXdYd;

		double z0 = z;
		z = z ? 1. / z : 1;
		x *= z; y *= z;

		r2 = x * x + y * y;
		r4 = r2 * r2;
		r6 = r4 * r2;
		a1 = 2 * x * y;
		a2 = r2 + 2 * x * x;
		a3 = r2 + 2 * y * y;
		cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;
		icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
		xd0 = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
		yd0 = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;

		// additional distortion by projecting onto a tilt plane
		vecTilt = matTilt * Vec3d(xd0, yd0, 1);
		invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
		xd = invProj * vecTilt(0);
		yd = invProj * vecTilt(1);

		m[i].x = xd * fx + cx;
		m[i].y = yd * fy + cy;

		if (calc_derivatives)
		{
			for (int row = 0; row < 2; ++row)
				for (int col = 0; col < 2; ++col)
					dMatTilt(row, col) = matTilt(row, col) * vecTilt(2)
					- matTilt(2, col) * vecTilt(row);
			double invProjSquare = (invProj * invProj);
			dMatTilt *= invProjSquare;

			if (dpdt_p)
			{
				double dxdt[] = { z, 0, -x * z }, dydt[] = { 0, z, -y * z };
				for (j = 0; j < 3; j++)
				{
					double dr2dt = 2 * x * dxdt[j] + 2 * y * dydt[j];
					double dcdist_dt = k[0] * dr2dt + 2 * k[1] * r2 * dr2dt + 3 * k[4] * r4 * dr2dt;
					double dicdist2_dt = -icdist2 * icdist2 * (k[5] * dr2dt + 2 * k[6] * r2 * dr2dt + 3 * k[7] * r4 * dr2dt);
					double da1dt = 2 * (x * dydt[j] + y * dxdt[j]);
					double dmxdt = (dxdt[j] * cdist * icdist2 + x * dcdist_dt * icdist2 + x * cdist * dicdist2_dt +
						k[2] * da1dt + k[3] * (dr2dt + 4 * x * dxdt[j]) + k[8] * dr2dt + 2 * r2 * k[9] * dr2dt);
					double dmydt = (dydt[j] * cdist * icdist2 + y * dcdist_dt * icdist2 + y * cdist * dicdist2_dt +
						k[2] * (dr2dt + 4 * y * dydt[j]) + k[3] * da1dt + k[10] * dr2dt + 2 * r2 * k[11] * dr2dt);
					dXdYd = dMatTilt * Vec2d(dmxdt, dmydt);
					dpdt_p[j] = fx * dXdYd(0);
					dpdt_p[dpdt_step + j] = fy * dXdYd(1);
				}
				dpdt_p += dpdt_step * 2;
			}

			if (dpdr_p)
			{
				double dx0dr[] =
				{
					X * dRdr[0] + Y * dRdr[1] + Z * dRdr[2],
					X * dRdr[9] + Y * dRdr[10] + Z * dRdr[11],
					X * dRdr[18] + Y * dRdr[19] + Z * dRdr[20]
				};
				double dy0dr[] =
				{
					X * dRdr[3] + Y * dRdr[4] + Z * dRdr[5],
					X * dRdr[12] + Y * dRdr[13] + Z * dRdr[14],
					X * dRdr[21] + Y * dRdr[22] + Z * dRdr[23]
				};
				double dz0dr[] =
				{
					X * dRdr[6] + Y * dRdr[7] + Z * dRdr[8],
					X * dRdr[15] + Y * dRdr[16] + Z * dRdr[17],
					X * dRdr[24] + Y * dRdr[25] + Z * dRdr[26]
				};
				for (j = 0; j < 3; j++)
				{
					double dxdr = z * (dx0dr[j] - x * dz0dr[j]);
					double dydr = z * (dy0dr[j] - y * dz0dr[j]);
					double dr2dr = 2 * x * dxdr + 2 * y * dydr;
					double dcdist_dr = (k[0] + 2 * k[1] * r2 + 3 * k[4] * r4) * dr2dr;
					double dicdist2_dr = -icdist2 * icdist2 * (k[5] + 2 * k[6] * r2 + 3 * k[7] * r4) * dr2dr;
					double da1dr = 2 * (x * dydr + y * dxdr);
					double dmxdr = (dxdr * cdist * icdist2 + x * dcdist_dr * icdist2 + x * cdist * dicdist2_dr +
						k[2] * da1dr + k[3] * (dr2dr + 4 * x * dxdr) + (k[8] + 2 * r2 * k[9]) * dr2dr);
					double dmydr = (dydr * cdist * icdist2 + y * dcdist_dr * icdist2 + y * cdist * dicdist2_dr +
						k[2] * (dr2dr + 4 * y * dydr) + k[3] * da1dr + (k[10] + 2 * r2 * k[11]) * dr2dr);
					dXdYd = dMatTilt * Vec2d(dmxdr, dmydr);
					dpdr_p[j] = fx * dXdYd(0);
					dpdr_p[dpdr_step + j] = fy * dXdYd(1);
				}
				dpdr_p += dpdr_step * 2;
			}

		}
	}


	cvConvert(_m, imagePoints);

	if (_dpdr != dpdr)
		cvConvert(_dpdr, dpdr);

	if (_dpdt != dpdt)
		cvConvert(_dpdt, dpdt);
}



/*
void cvConvertPointsHomogeneous(const CvMat* _src, CvMat* _dst)
{
	cv::Mat src = cv::cvarrToMat(_src), dst = cv::cvarrToMat(_dst);
	const cv::Mat dst0 = dst;

	dst = src.reshape(dst0.channels(), dst0.rows);

	
	dst.convertTo(dst0, dst0.type());
}
*/
/*
void __undistortPoints(InputArray _src, OutputArray _dst, InputArray _cameraMatrix)
{

	Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();

	_dst.create(src.rows, 1, CV_32FC2);

	Mat dst = _dst.getMat();

	const CvPoint2D32f* srcf = (const CvPoint2D32f*)src.data;
	CvPoint2D32f* dstf = (CvPoint2D32f*)dst.data;

	/*****************************************************
	int stype = CV_MAT_TYPE(src.type());
	int dtype = CV_MAT_TYPE(dst.type());
	int sstep = src.step / CV_ELEM_SIZE(stype);
	int dstep = dst.step / CV_ELEM_SIZE(dtype);
	/*****************************************************

	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);

	int n = src.rows + src.cols - 1;
	for (int i = 0; i < n; i++)
	{
		double x, y;
		x = srcf[i * sstep].x;
		y = srcf[i * sstep].y;

		x = (x - cx) / fx;
		y = (y - cy) / fy;

		dstf[i * dstep].x = (float)x;
		dstf[i * dstep].y = (float)y;
	}
}
*/

/*
void cvUndistortPoints(const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix, const CvMat* matP)
{
	const CvPoint2D64f* srcd = (const CvPoint2D64f*)_src->data.ptr;
	CvPoint2D64f* dstd = (CvPoint2D64f*)_dst->data.ptr;
	int stype = CV_MAT_TYPE(_src->type);
	int dtype = CV_MAT_TYPE(_dst->type);
	int sstep = 1;
	int dstep = 1;

	double fx = CV_MAT_ELEM(*_cameraMatrix, double, 0, 0);
	double fy = CV_MAT_ELEM(*_cameraMatrix, double, 1, 1);
	double cx = CV_MAT_ELEM(*_cameraMatrix, double, 0, 2);
	double cy = CV_MAT_ELEM(*_cameraMatrix, double, 1, 2);

	int n = _src->rows + _src->cols - 1;
	for (int i = 0; i < n; i++)
	{
		double x, y;
		x = srcd[i * sstep].x;
		y = srcd[i * sstep].y;

		x = (x - cx) * (1. / fx);
		y = (y - cy) * (1. / fy);

		dstd[i * dstep].x = x;
		dstd[i * dstep].y = y;
	}
}*/

/*
int cvFindHomography(const CvMat* _src, const CvMat* _dst, CvMat* __H, int method,
	double ransacReprojThreshold, CvMat* _mask, int maxIters,
	double confidence)
{
	cv::Mat src = cv::cvarrToMat(_src), dst = cv::cvarrToMat(_dst);

	if (src.channels() == 1 && (src.rows == 2 || src.rows == 3) && src.cols > 3)
		cv::transpose(src, src);
	if (dst.channels() == 1 && (dst.rows == 2 || dst.rows == 3) && dst.cols > 3)
		cv::transpose(dst, dst);

	if (maxIters < 0)
		maxIters = 0;
	if (maxIters > 2000)
		maxIters = 2000;

	if (confidence < 0)
		confidence = 0;
	if (confidence > 1)
		confidence = 1;

	const cv::Mat H = cv::cvarrToMat(__H), mask = cv::cvarrToMat(_mask);
	cv::Mat H0 = cv::findHomography(src, dst, method, ransacReprojThreshold,
		_mask ? cv::_OutputArray(mask) : cv::_OutputArray(), maxIters,
		confidence);

	if (H0.empty())
	{
		cv::Mat Hz = cv::cvarrToMat(__H);
		Hz.setTo(cv::Scalar::all(0));
		return 0;
	}
	H0.convertTo(H, H.type());
	return 1;
}
*/
/*
void cvFindExtrinsicCameraParams2(const CvMat* objectPoints,
	const CvMat* imagePoints, const CvMat* A, CvMat* rvec, CvMat* tvec)
{
	const int max_iter = 20;
	Ptr<CvMat> matM, _Mxy, _m, _mn, matL;

	int i, count;
	double a[9], R[9];
	double MM[9] = { 0 }, U[9] = { 0 }, V[9] = { 0 }, W[3] = { 0 };
	cv::Scalar Mc;
	double param[6] = { 0 };
	CvMat matA = cvMat(3, 3, CV_64F, a);
	CvMat matR = cvMat(3, 3, CV_64F, R);
	CvMat _r = cvMat(3, 1, CV_64F, param);
	CvMat _t = cvMat(3, 1, CV_64F, param + 3);
	CvMat _Mc = cvMat(1, 3, CV_64F, Mc.val);
	CvMat _MM = cvMat(3, 3, CV_64F, MM);
	CvMat matU = cvMat(3, 3, CV_64F, U);
	CvMat matV = cvMat(3, 3, CV_64F, V);
	CvMat matW = cvMat(3, 1, CV_64F, W);
	CvMat _param = cvMat(6, 1, CV_64F, param);
	CvMat _dpdr, _dpdt;


	count = MAX(objectPoints->cols, objectPoints->rows);
	matM.reset(cvCreateMat(1, count, CV_64FC3));
	_m.reset(cvCreateMat(1, count, CV_64FC2));

	//cvConvertPointsHomogeneous(objectPoints, matM); 
	{
		Mat src = cvarrToMat(objectPoints), dst = cv::cvarrToMat(matM);
		const Mat dst0 = dst;

		dst = src.reshape(dst0.channels(), dst0.rows);


		dst.convertTo(dst0, dst0.type());
	}
	//cvConvertPointsHomogeneous(imagePoints, _m); 
	{
		Mat src = cvarrToMat(imagePoints), dst = cv::cvarrToMat(_m);
		const Mat dst0 = dst;

		dst = src.reshape(dst0.channels(), dst0.rows);


		dst.convertTo(dst0, dst0.type());
	}
	cvConvert(A, &matA);

	
	_mn.reset(cvCreateMat(1, count, CV_64FC2));
	_Mxy.reset(cvCreateMat(1, count, CV_64FC2));

	
	//cvUndistortPoints(_m, _mn, &matA, &_Ar);
	const CvPoint2D64f* srcd = (const CvPoint2D64f*)_m->data.ptr;
	CvPoint2D64f* dstd = (CvPoint2D64f*)_mn->data.ptr;
	int stype = CV_MAT_TYPE(_m->type);
	int dtype = CV_MAT_TYPE(_mn->type);
	int sstep = 1;
	int dstep = 1;

	double fx = CV_MAT_ELEM(matA, double, 0, 0);
	double fy = CV_MAT_ELEM(matA, double, 1, 1);
	double cx = CV_MAT_ELEM(matA, double, 0, 2);
	double cy = CV_MAT_ELEM(matA, double, 1, 2);

	int n = _m->rows + _m->cols - 1;
	for (int i = 0; i < n; i++)
	{
		double x, y;
		x = srcd[i * sstep].x;
		y = srcd[i * sstep].y;

		x = (x - cx) * (1. / fx);
		y = (y - cy) * (1. / fy);

		dstd[i * dstep].x = x;
		dstd[i * dstep].y = y;
	}

	Mc = cvAvg(matM);
	cvReshape(matM, matM, 1, count);
	cvMulTransposed(matM, &_MM, 1, &_Mc);
	__cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);

	// non-planar structure. Use DLT method
	double* L;
	double LL[12 * 12], LW[12], LV[12 * 12], sc;
	CvMat _LL = cvMat(12, 12, CV_64F, LL);
	CvMat _LW = cvMat(12, 1, CV_64F, LW);
	CvMat _LV = cvMat(12, 12, CV_64F, LV);
	CvMat _RRt, _RR, _tt;
	CvPoint3D64f* M = (CvPoint3D64f*)matM->data.db;
	CvPoint2D64f* mn = (CvPoint2D64f*)_mn->data.db;

	matL.reset(cvCreateMat(2 * count, 12, CV_64F));
	L = matL->data.db;

	for (i = 0; i < count; i++, L += 24)
	{
		double x = -mn[i].x, y = -mn[i].y;
		L[0] = L[16] = M[i].x;
		L[1] = L[17] = M[i].y;
		L[2] = L[18] = M[i].z;
		L[3] = L[19] = 1.;
		L[4] = L[5] = L[6] = L[7] = 0.;
		L[12] = L[13] = L[14] = L[15] = 0.;
		L[8] = x * M[i].x;
		L[9] = x * M[i].y;
		L[10] = x * M[i].z;
		L[11] = x;
		L[20] = y * M[i].x;
		L[21] = y * M[i].y;
		L[22] = y * M[i].z;
		L[23] = y;
	}

	cvMulTransposed(matL, &_LL, 1);
	__cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
	_RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
	cvGetCols(&_RRt, &_RR, 0, 3);
	cvGetCol(&_RRt, &_tt, 3);

	if (cvDet(&_RR) < 0) {
		cvScale(&_RRt, &_RRt, -1);
	}

	sc = cvNorm(&_RR);
	__cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
	//cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
	matR = cvMat(Mat(matU.rows, matU.cols, matU.type, matU.data.db).t() * Mat(matV.rows, matV.cols, matV.type, matV.data.db));
	cvScale(&_tt, &_t, cvNorm(&matR) / sc);
	cvRodrigues2(&matR, &_r, 0);

	cvReshape(matM, matM, 3, 1);
	cvReshape(_mn, _mn, 2, 1);

	// refine extrinsic parameters using iterative algorithm
	CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, FLT_EPSILON), true);
	cvCopy(&_param, solver.param);

	for (;;)
	{
		CvMat* matJ = 0, * _err = 0;
		const CvMat* __param = 0;
		bool proceed = solver.update(__param, matJ, _err);
		cvCopy(__param, &_param);
		if (!proceed || !_err)
			break;
		cvReshape(_err, _err, 2, 1);
		if (matJ)
		{
			cvGetCols(matJ, &_dpdr, 0, 3);
			cvGetCols(matJ, &_dpdt, 3, 6);
			cvProjectPoints2(matM, &_r, &_t, &matA, 0,
				_err, &_dpdr, &_dpdt);
		}
		else
		{
			cvProjectPoints2(matM, &_r, &_t, &matA, 0,
				_err, 0, 0);
		}
		cvSub(_err, _m, _err);
		cvReshape(_err, _err, 1, 2 * count);
	}
	cvCopy(solver.param, &_param);


	_r = cvMat(rvec->rows, rvec->cols, CV_64F, param);
	_t = cvMat(tvec->rows, tvec->cols, CV_64F, param + 3);

	cvConvert(&_r, rvec);
	cvConvert(&_t, tvec);
}
*/





int __solvePnPGeneric__(InputArray _opoints, InputArray _ipoints,
	InputArray _cameraMatrix, InputArray _distCoeffs,
	OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs, SolvePnPMethod flags,
	InputArray _rvec, InputArray _tvec,
	OutputArray reprojectionError)
{

	Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
	int npoints = max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));

	opoints = opoints.reshape(3, npoints);
	ipoints = ipoints.reshape(2, npoints);



	Mat cameraMatrix0 = _cameraMatrix.getMat();
	Mat distCoeffs0 = _distCoeffs.getMat();
	Mat cameraMatrix = Mat_<double>(cameraMatrix0);
	Mat distCoeffs = Mat_<double>(distCoeffs0);

	vector<Mat> vec_rvecs, vec_tvecs;
	if (flags == SOLVEPNP_EPNP)
	{
		Mat undistortedPoints;
		//undistortPoints(ipoints, undistortedPoints, cameraMatrix);
		{
			undistortedPoints.create(ipoints.rows, 1, CV_32FC2);

			const CvPoint2D32f* srcf = (const CvPoint2D32f*)ipoints.data;
			CvPoint2D32f* dstf = (CvPoint2D32f*)undistortedPoints.data;

			/*****************************************************/
			int stype = CV_MAT_TYPE(ipoints.type());
			int dtype = CV_MAT_TYPE(undistortedPoints.type());
			int sstep = ipoints.step / CV_ELEM_SIZE(stype);
			int dstep = undistortedPoints.step / CV_ELEM_SIZE(dtype);
			/*****************************************************/

			double fx = cameraMatrix.at<double>(0, 0);
			double fy = cameraMatrix.at<double>(1, 1);
			double cx = cameraMatrix.at<double>(0, 2);
			double cy = cameraMatrix.at<double>(1, 2);

			int n = ipoints.rows + ipoints.cols - 1;
			for (int i = 0; i < n; i++)
			{
				double x, y;
				x = srcf[i * sstep].x;
				y = srcf[i * sstep].y;

				x = (x - cx) / fx;
				y = (y - cy) / fy;

				dstf[i * dstep].x = (float)x;
				dstf[i * dstep].y = (float)y;
			}
		}

		epnp PnP(cameraMatrix, opoints, undistortedPoints);
		/* {
			double uc, vc, fu, fv;
			std::vector<double> pws, us, alphas, pcs;
			int number_of_correspondences;

			double cws[4][3], ccs[4][3];
			int max_nr;
			double* A1, * A2;


			uc = cameraMatrix.at<double>(0, 2);
			vc = cameraMatrix.at<double>(1, 2);
			fu = cameraMatrix.at<double>(0, 0);
			fv = cameraMatrix.at<double>(1, 1);

			number_of_correspondences = opoints.checkVector(3, CV_32F);
			pws.resize(3 * number_of_correspondences);
			us.resize(2 * number_of_correspondences);

			//init_points<Point3f, Point2f>(opoints, ipoints);
			for (int i = 0; i < number_of_correspondences; i++)
			{
				pws[3 * i] = opoints.at<Point3f>(i).x;
				pws[3 * i + 1] = opoints.at<Point3f>(i).y;
				pws[3 * i + 2] = opoints.at<Point3f>(i).z;

				us[2 * i] = ipoints.at<Point2f>(i).x * fu + uc;
				us[2 * i + 1] = ipoints.at<Point2f>(i).y * fv + vc;
			}

			alphas.resize(4 * number_of_correspondences);
			pcs.resize(3 * number_of_correspondences);

			max_nr = 0;
			A1 = NULL;
			A2 = NULL;
		}*/

		Mat rvec, tvec, R;
		PnP.compute_pose(R, tvec);


		Rodrigues(R, rvec);

		vec_rvecs.push_back(rvec);
		vec_tvecs.push_back(tvec);
	}
	
	if (flags == SOLVEPNP_ITERATIVE)
	{
		Mat rvec, tvec;
		rvec.create(3, 1, CV_64FC1);
		tvec.create(3, 1, CV_64FC1);

		CvMat c_objectPoints = cvMat(opoints), c_imagePoints = cvMat(ipoints);
		CvMat c_cameraMatrix = cvMat(cameraMatrix), c_distCoeffs = cvMat(distCoeffs);
		CvMat c_rvec = cvMat(rvec), c_tvec = cvMat(tvec);
		//cvFindExtrinsicCameraParams2(&c_objectPoints, &c_imagePoints, &c_cameraMatrix,
			//&c_rvec, &c_tvec);
		//void cvFindExtrinsicCameraParams2(const CvMat * objectPoints,
			//const CvMat * imagePoints, const CvMat * A, CvMat * rvec, CvMat * tvec)
		{
			const int max_iter = 20;
			Ptr<CvMat> matM, _Mxy, _m, _mn, matL;

			int i, count;
			double a[9], R[9];
			double MM[9] = { 0 }, U[9] = { 0 }, V[9] = { 0 }, W[3] = { 0 };
			Scalar Mc;
			double param[6] = { 0 };
			CvMat matA = cvMat(3, 3, CV_64F, a);
			CvMat matR = cvMat(3, 3, CV_64F, R);
			CvMat _r = cvMat(3, 1, CV_64F, param);
			CvMat _t = cvMat(3, 1, CV_64F, param + 3);
			CvMat _Mc = cvMat(1, 3, CV_64F, Mc.val);
			CvMat _MM = cvMat(3, 3, CV_64F, MM);
			CvMat matU = cvMat(3, 3, CV_64F, U);
			CvMat matV = cvMat(3, 3, CV_64F, V);
			CvMat matW = cvMat(3, 1, CV_64F, W);
			CvMat _param = cvMat(6, 1, CV_64F, param);
			CvMat _dpdr, _dpdt;


			count = MAX(c_objectPoints.cols, c_objectPoints.rows);
			matM.reset(cvCreateMat(1, count, CV_64FC3));
			_m.reset(cvCreateMat(1, count, CV_64FC2));

			//cvConvertPointsHomogeneous(objectPoints, matM); 
			{
				Mat dst = cvarrToMat(matM);
				const Mat dst0 = dst;

				dst = opoints.reshape(dst0.channels(), dst0.rows);


				dst.convertTo(dst0, dst0.type());
			}
			//cvConvertPointsHomogeneous(imagePoints, _m); 
			{
				Mat dst = cvarrToMat(_m);
				const Mat dst0 = dst;

				dst = ipoints.reshape(dst0.channels(), dst0.rows);


				dst.convertTo(dst0, dst0.type());
			}
			cvConvert(&c_cameraMatrix, &matA);


			_mn.reset(cvCreateMat(1, count, CV_64FC2));
			_Mxy.reset(cvCreateMat(1, count, CV_64FC2));


			//cvUndistortPoints(_m, _mn, &matA, &_Ar);
			const CvPoint2D64f* srcd = (const CvPoint2D64f*)_m->data.ptr;
			CvPoint2D64f* dstd = (CvPoint2D64f*)_mn->data.ptr;
			int stype = CV_MAT_TYPE(_m->type);
			int dtype = CV_MAT_TYPE(_mn->type);
			int sstep = 1;
			int dstep = 1;

			double fx = CV_MAT_ELEM(matA, double, 0, 0);
			double fy = CV_MAT_ELEM(matA, double, 1, 1);
			double cx = CV_MAT_ELEM(matA, double, 0, 2);
			double cy = CV_MAT_ELEM(matA, double, 1, 2);

			int n = _m->rows + _m->cols - 1;
			for (int i = 0; i < n; i++)
			{
				double x, y;
				x = srcd[i * sstep].x;
				y = srcd[i * sstep].y;

				x = (x - cx) * (1. / fx);
				y = (y - cy) * (1. / fy);

				dstd[i * dstep].x = x;
				dstd[i * dstep].y = y;
			}

			Mc = cvAvg(matM);
			cvReshape(matM, matM, 1, count);
			cvMulTransposed(matM, &_MM, 1, &_Mc);
			__cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);

			// non-planar structure. Use DLT method
			double* L;
			double LL[12 * 12], LW[12], LV[12 * 12], sc;
			CvMat _LL = cvMat(12, 12, CV_64F, LL);
			CvMat _LW = cvMat(12, 1, CV_64F, LW);
			CvMat _LV = cvMat(12, 12, CV_64F, LV);
			CvMat _RRt, _RR, _tt;
			CvPoint3D64f* M = (CvPoint3D64f*)matM->data.db;
			CvPoint2D64f* mn = (CvPoint2D64f*)_mn->data.db;

			matL.reset(cvCreateMat(2 * count, 12, CV_64F));
			L = matL->data.db;

			for (i = 0; i < count; i++, L += 24)
			{
				double x = -mn[i].x, y = -mn[i].y;
				L[0] = L[16] = M[i].x;
				L[1] = L[17] = M[i].y;
				L[2] = L[18] = M[i].z;
				L[3] = L[19] = 1.;
				L[4] = L[5] = L[6] = L[7] = 0.;
				L[12] = L[13] = L[14] = L[15] = 0.;
				L[8] = x * M[i].x;
				L[9] = x * M[i].y;
				L[10] = x * M[i].z;
				L[11] = x;
				L[20] = y * M[i].x;
				L[21] = y * M[i].y;
				L[22] = y * M[i].z;
				L[23] = y;
			}

			cvMulTransposed(matL, &_LL, 1);
			__cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
			_RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
			cvGetCols(&_RRt, &_RR, 0, 3);
			cvGetCol(&_RRt, &_tt, 3);

			if (cvDet(&_RR) < 0) {
				cvScale(&_RRt, &_RRt, -1);
			}

			sc = cvNorm(&_RR);
			__cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
			//cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
			matR = cvMat(Mat(matU.rows, matU.cols, matU.type, matU.data.db).t() * Mat(matV.rows, matV.cols, matV.type, matV.data.db));
			cvScale(&_tt, &_t, cvNorm(&matR) / sc);
			cvRodrigues2(&matR, &_r, 0);

			cvReshape(matM, matM, 3, 1);
			cvReshape(_mn, _mn, 2, 1);

			// refine extrinsic parameters using iterative algorithm
			CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, FLT_EPSILON), true);
			cvCopy(&_param, solver.param);

			for (;;)
			{
				CvMat* matJ = 0, * _err = 0;
				const CvMat* __param = 0;
				bool proceed = solver.update(__param, matJ, _err);
				cvCopy(__param, &_param);
				if (!proceed || !_err)
					break;
				cvReshape(_err, _err, 2, 1);
				if (matJ)
				{
					cvGetCols(matJ, &_dpdr, 0, 3);
					cvGetCols(matJ, &_dpdt, 3, 6);
					cvProjectPoints2(matM, &_r, &_t, &matA, 0,
						_err, &_dpdr, &_dpdt);
				}
				else
				{
					cvProjectPoints2(matM, &_r, &_t, &matA, 0,
						_err, 0, 0);
				}
				cvSub(_err, _m, _err);
				cvReshape(_err, _err, 1, 2 * count);
			}
			cvCopy(solver.param, &_param);


			_r = cvMat(c_rvec.rows, c_rvec.cols, CV_64F, param);
			_t = cvMat(c_tvec.rows, c_tvec.cols, CV_64F, param + 3);

			cvConvert(&_r, &c_rvec);
			cvConvert(&_t, &c_tvec);
		}




		vec_rvecs.push_back(rvec);
		vec_tvecs.push_back(tvec);
	}

	int solutions = (int)(vec_rvecs.size());

	_rvecs.create(solutions, 1, CV_64F);
	_tvecs.create(solutions, 1, CV_64F);


	for (int i = 0; i < solutions; i++)
	{
		_rvecs.getMatRef(i) = vec_rvecs[i];
		_tvecs.getMatRef(i) = vec_tvecs[i];
	}

	

	return solutions;
}




bool __solvePnP__(InputArray opoints, InputArray ipoints,
	InputArray cameraMatrix, InputArray distCoeffs,
	OutputArray rvec, OutputArray tvec, int flags)
{

	vector<Mat> rvecs, tvecs;
	int solutions = __solvePnPGeneric__(opoints, ipoints, cameraMatrix, distCoeffs, rvecs, tvecs, (SolvePnPMethod)flags, rvec, tvec, noArray());

	if (solutions > 0)
	{
		int rdepth = rvec.empty() ? CV_64F : rvec.depth();
		int tdepth = tvec.empty() ? CV_64F : tvec.depth();
		rvecs[0].convertTo(rvec, rdepth);
		tvecs[0].convertTo(tvec, tdepth);
	}

	return solutions > 0;
}




bool Methods::__solvePnPRansac__(Mat opoints, Mat ipoints,
	Mat cameraMatrix, Mat distCoeffs,
	Mat& rvec, Mat& tvec, bool useExtrinsicGuess,
	int iterationsCount, float reprojectionError, double confidence,
	int flags)
{
	int npoints = opoints.rows;

	rvec = Mat(3, 1, CV_64FC1);
	tvec = Mat(3, 1, CV_64FC1);

	int modelPoints = 5;
	int ransac_kernel_method = SOLVEPNP_EPNP;


	double param1 = reprojectionError;                // reprojection error
	double param2 = confidence;                       // confidence
	int param3 = iterationsCount;                     // number maximum iterations

	Mat _local_model(3, 2, CV_64FC1);
	Mat _mask_local_inliers(1, opoints.rows, CV_8UC1);

	// call Ransac
	//int result = __createRANSACPointSetRegistrator__(cb, modelPoints,
		//param1, param2, param3)->__run__(opoints, ipoints, _local_model, _mask_local_inliers);


	bool result = false;
	{
		OutputArray _model = _local_model;
		OutputArray _mask = _mask_local_inliers;
		Mat err, mask, model, bestModel, ms1, ms2;

		int iter, niters = MAX(iterationsCount, 1);
		int d1 = opoints.cols;
		int count = opoints.rows, maxGoodCount = 0;

		RNG rng((uint64)-1);

		if (count < modelPoints)
			return false;

		Mat bestMask0, bestMask;

		_mask.create(count, 1, CV_8U, -1, true);
		bestMask0 = bestMask = _mask.getMat();

		for (iter = 0; iter < niters; iter++)
		{
			int i, nmodels;

			//bool found = __getSubset__(m1, m2, ms1, ms2, rng, 10000);
			{
				cv::AutoBuffer<int> _idx(modelPoints);
				int* idx = _idx.data();

				const int d2 = ipoints.channels();

				int esz1 = (int)opoints.elemSize1() * d1;
				int esz2 = (int)ipoints.elemSize1() * d2;
				esz1 /= sizeof(int);
				esz2 /= sizeof(int);

				const int* m1ptr = opoints.ptr<int>();
				const int* m2ptr = ipoints.ptr<int>();

				ms1.create(modelPoints, 1, CV_32FC3);
				ms2.create(modelPoints, 1, CV_32FC2);


				int* ms1ptr = ms1.ptr<int>();
				int* ms2ptr = ms2.ptr<int>();

				for (int iters = 0; iters < 1000; ++iters)
				{
					int i;

					for (i = 0; i < modelPoints; ++i)
					{
						int idx_i;

						for (idx_i = rng.uniform(0, count);
							std::find(idx, idx + i, idx_i) != idx + i;
							idx_i = rng.uniform(0, count))
						{
						}

						idx[i] = idx_i;

						for (int k = 0; k < esz1; ++k)
							ms1ptr[i * esz1 + k] = m1ptr[idx_i * esz1 + k];

						for (int k = 0; k < esz2; ++k)
							ms2ptr[i * esz2 + k] = m2ptr[idx_i * esz2 + k];
					}
					break;
				}
			}
			//nmodels = cb->__runKernel__(ms1, ms2, model);
			{
				nmodels = __solvePnP__(ms1, ms2, cameraMatrix, distCoeffs,
					rvec, tvec, ransac_kernel_method);

				Mat _local_model;
				hconcat(rvec, tvec, _local_model);
				_local_model.copyTo(model);
			}
			Size modelSize(model.cols, model.rows / nmodels);


			for (i = 0; i < nmodels; i++)
			{
				Mat model_i = model.rowRange(i * modelSize.height, (i + 1) * modelSize.height);
				//int goodCount = __findInliers__(m1, m2, model_i, err, mask, threshold);
				int goodCount;
				{
					//cb->__computeError__(m1, m2, model, err);
					{
						int i, count = opoints.checkVector(3);
						Mat _rvec = model_i.col(0);
						Mat _tvec = model_i.col(1);


						Mat projpoints(count, 2, CV_32FC1);
						projectPoints(opoints, _rvec, _tvec, cameraMatrix, distCoeffs, projpoints);

						const Point2f* ipoints_ptr = ipoints.ptr<Point2f>();
						const Point2f* projpoints_ptr = projpoints.ptr<Point2f>();

						err.create(count, 1, CV_32FC1);

						for (i = 0; i < count; ++i)
							err.ptr<float>()[i] = (float)norm(Matx21f(ipoints_ptr[i] - projpoints_ptr[i]), NORM_L2SQR);
					}
					mask.create(err.size(), CV_8U);

					const float* errptr = err.ptr<float>();
					uchar* maskptr = mask.ptr<uchar>();
					float t = (float)(reprojectionError * reprojectionError);
					int i, n = (int)err.total();
					goodCount = 0;
					for (i = 0; i < n; i++)
					{
						int f = errptr[i] <= t;
						maskptr[i] = (uchar)f;
						goodCount += f;
					}
				}

				if (goodCount > MAX(maxGoodCount, modelPoints - 1))
				{
					std::swap(mask, bestMask);
					model_i.copyTo(bestModel);
					maxGoodCount = goodCount;
					//niters = RANSACUpdateNumIters(confidence, (double)(count - goodCount) / count, modelPoints, niters);
					{
						double ep = (double)(count - goodCount) / count;
						double p = confidence;
						p = MAX(p, 0.);
						p = MIN(p, 1.);
						ep = MAX(ep, 0.);
						ep = MIN(ep, 1.);

						// avoid inf's & nan's
						double num = MAX(1. - p, DBL_MIN);
						double denom = 1. - std::pow(1. - ep, modelPoints);
						if (denom < DBL_MIN)
							niters = 0;

						num = std::log(num);
						denom = std::log(denom);

						niters = denom >= 0 || -num >= niters * (-denom) ? niters : cvRound(num / denom);
					}
					

				}
			}
		}

		transpose(bestMask, bestMask0);
		bestModel.copyTo(_model);
		result = true;
	}
label:

	vector<Point3d> opoints_inliers;
	vector<Point2d> ipoints_inliers;
	opoints = opoints.reshape(3);
	ipoints = ipoints.reshape(2);
	opoints.convertTo(opoints_inliers, CV_64F);
	ipoints.convertTo(ipoints_inliers, CV_64F);

	const uchar* mask = _mask_local_inliers.ptr<uchar>();
	int npoints1;// = __compressElems__(&opoints_inliers[0], mask, 1, npoints);
	//(T* ptr, const uchar* mask, int mstep, int count)
	{
		int i, j;
		for (i = j = 0; i < npoints; i++)
			if (mask[i])
			{
				if (i > j)
					opoints_inliers[j] = opoints_inliers[i];
				j++;
			}
		npoints1 = j;
	}
	//__compressElems__(&ipoints_inliers[0], mask, 1, npoints);
	{
		int i, j;
		for (i = j = 0; i < npoints; i++)
			if (mask[i])
			{
				if (i > j)
					ipoints_inliers[j] = ipoints_inliers[i];
				j++;
			}
	}

	opoints_inliers.resize(npoints1);
	ipoints_inliers.resize(npoints1);
	result = __solvePnP__(opoints_inliers, ipoints_inliers, cameraMatrix,
		distCoeffs, rvec, tvec, flags) ? 1 : -1;

	return true;
}
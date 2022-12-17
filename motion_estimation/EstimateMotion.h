#pragma once
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d/calib3d_c.h>
#include<opencv2/core/core_c.h>
#include<vector>
#include<iostream>

//using namespace cv;
using namespace std;

class EstimateMotion
{
private:
	bool __solvePnPRansac__(cv::Mat opoints, cv::Mat ipoints,
        cv::Mat cameraMatrix, cv::Mat distCoeffs,
        cv::Mat& _rvec, cv::Mat& _tvec, bool useExtrinsicGuess,
		int iterationsCount, float reprojectionError, double confidence,
		int flags);

	bool __solvePnP__(cv::InputArray opoints, cv::InputArray ipoints,
        cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
        cv::OutputArray rvec, cv::OutputArray tvec, int flags);

	int __solvePnPGeneric__(cv::InputArray _opoints, cv::InputArray _ipoints,
        cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
        cv::OutputArrayOfArrays _rvecs, cv::OutputArrayOfArrays _tvecs, cv::SolvePnPMethod flags,
        cv::InputArray _rvec, cv::InputArray _tvec,
        cv::OutputArray reprojectionError);


    void __cvSVD(CvArr* aarr, CvArr* warr, CvArr* uarr, CvArr* varr, int flags);

    void _SVDcompute(cv::InputArray _aarr, cv::OutputArray _w,
        cv::OutputArray _u, cv::OutputArray _vt);

    void __JacobiSVD(double* At, size_t astep, double* _W, double* Vt,
        size_t vstep, int m, int n, int n11 = -1);

    int cvRodrigues2(const CvMat* src, CvMat* dst, CvMat* jacobian);

    void cvProjectPoints2(const CvMat* objectPoints,
        const CvMat* r_vec,
        const CvMat* t_vec,
        const CvMat* A,
        const CvMat* distCoeffs,
        CvMat* imagePoints, CvMat* dpdr,
        CvMat* dpdt);

    void __Rodrigues(cv::InputArray _src, cv::OutputArray _dst, cv::OutputArray _jacobian);
public:
	void estimate_motion(vector<vector<cv::DMatch>> match, vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2,
        cv::Mat k, cv::Mat depth1, int max_depth,
        cv::Mat& rmat, cv::Mat& tvec, cv::Mat& image1_points, cv::Mat& image2_points,
		int maxIter, float error, float confidence);
};


struct SVDD
{
    cv::Mat u;
    cv::Mat vt;
    cv::Mat w;
};



namespace cv
{

    class epnp
    {
    public:
        epnp(const cv::Mat& cameraMatrix, const cv::Mat& opoints, const cv::Mat& ipoints);
        ~epnp();

        void compute_pose(cv::Mat& R, cv::Mat& t);

    private:
        epnp(const epnp&); // copy disabled

        double dot(const double* v1, const double* v2);

        double compute_R_and_t(const double* ut, const double* betas,
            double R[3][3], double t[3]);

        void gauss_newton(const CvMat* L_6x10, const CvMat* Rho, double current_betas[4]);

        template <typename T>
        void init_camera_parameters(const cv::Mat& cameraMatrix)
        {
            uc = cameraMatrix.at<T>(0, 2);
            vc = cameraMatrix.at<T>(1, 2);
            fu = cameraMatrix.at<T>(0, 0);
            fv = cameraMatrix.at<T>(1, 1);
        }

        template <typename OpointType, typename IpointType>
        void init_points(const cv::Mat& opoints, const cv::Mat& ipoints)
        {
            for (int i = 0; i < number_of_correspondences; i++)
            {
                pws[3 * i] = opoints.at<OpointType>(i).x;
                pws[3 * i + 1] = opoints.at<OpointType>(i).y;
                pws[3 * i + 2] = opoints.at<OpointType>(i).z;

                us[2 * i] = ipoints.at<IpointType>(i).x * fu + uc;
                us[2 * i + 1] = ipoints.at<IpointType>(i).y * fv + vc;
            }
        }

        double uc, vc, fu, fv;

        std::vector<double> pws, us, alphas, pcs;
        int number_of_correspondences;

        double cws[4][3], ccs[4][3];
        int max_nr;
        double* A1, * A2;
    };

}


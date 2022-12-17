
#include "opencv2/core/core_c.h"
#include <iostream>

namespace cv
{

    class epnp
    {
    public:
        epnp(const cv::Mat &cameraMatrix, const cv::Mat &opoints, const cv::Mat &ipoints);
        ~epnp();

        void compute_pose(cv::Mat &R, cv::Mat &t);

    private:
        double dot(const double *v1, const double *v2);

        double compute_R_and_t(const double *ut, const double *betas,
                               double R[3][3], double t[3]);

        void gauss_newton(const CvMat *L_6x10, const CvMat *Rho, double current_betas[4]);

        double uc, vc, fu, fv;

        std::vector<double> pws, us, alphas, pcs;
        int number_of_correspondences;

        double cws[4][3], ccs[4][3];
        int max_nr;
        double *A1, *A2;
    };

}

#endif

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "Methods.h"
#include "Dataset_Handler.h"

using namespace cv;
using namespace std;

void tutorial(Methods method);

int main()
{
    Dataset_Handler handler(0);
    Methods method;

    int tut = false;
    if (tut)
        tutorial(method);

    /***************************************************************************************
        Down here is a visual odometry function which combine all the other functions
        mention in the tutorial.

        Some parameter can be modified such as detector, matching, filter_match_distance...
        whereas the mask is not recommended to modified.

        To be careful that it may takes pretty long time to finish processing on your PC,
        so please make sure everything works fine in small subset before you launch the
        full dataset, which contains 4000 and more images.

        The finxxxxal result will be saved as 'trajectory_nolidar_bm.txt' in the result folder
        in the arrangement of positions (x, y, z). You can use the Matlab script offered,
        or feel free to use your own test bench.
    ***************************************************************************************/
    int detector = Sift;
    bool matching = BF;
    float filter_match_distance = 0.45;
    bool stereo_matcher = SGBM;
    int subset = 0;

    Mat mask = Mat::zeros(handler.imheight, handler.imwidth, CV_8U);
    mask(Rect(96, 0, handler.imwidth - 96, handler.imheight)) = 255;

    vector<Mat> trajectory_nolidar_bm =
        method.visual_odometry(handler, detector, matching, filter_match_distance, stereo_matcher, subset, mask);

    FILE *fp;
    fopen_s(&fp, "../output/trajectory_nolidar_bm.txt", "w");

    for (int i = 0; i < trajectory_nolidar_bm.size(); i++)
    {
        fprintf_s(fp, "%lf\t%lf\t%lf\n",
                  trajectory_nolidar_bm.at(i).at<double>(0, 3),
                  trajectory_nolidar_bm.at(i).at<double>(1, 3),
                  trajectory_nolidar_bm.at(i).at<double>(2, 3));
    }

    fclose(fp);
}

void tutorial(Methods method)
{
    /*
     *	Part I - Data Exploration
     */

    /***********************************************************************************
        We get the ground truth poses of the stereo camera from the poses data in
        "../dataset/poses/00.txt"

        The poses is stored in vector (array) 'poses', which each pose is represented
        by a 3x4 transfomration matrix.

        Using Functions:
            groundTruthTrajectory()

    ***********************************************************************************/
    vector<cv::Mat> poses = method.groundTruthTrajectory("../dataset/poses/00.txt", 4541);

    printf("/*******************************************\n");
    printf("First 3 ground truth poses matrix:\n\n");
    for (int i = 0; i < 3; i++)
        cout << "pose " << i << ": " << endl
             << " " << poses[i] << endl
             << endl;

    system("pause");
    system("cls");

    /***********************************************************************************
        We can load in our sensor calibration data from the poses data in
        "../dataset/sequences/00/calib.txt"

        We can see that they have provided us 3x4 projection matrices for 4 cameras,
        as well as the transformation matrix for the LIDAR labeled Tr.

        We will need calibration data of camera0 and camera1, since we are going to
        handle the dataset from these 2 cameras.

        Using Functions:
            read_calib()

    ***********************************************************************************/
    Mat P0, P1;
    method.read_calib("../dataset/sequences/00/calib.txt", &P0, &P1);

    printf("/*******************************************\n");
    printf("3x4 projection matix of camera0 and camera1:\n\n");
    cout << "P0: " << endl
         << " " << P0 << endl
         << endl;
    cout << "P1: " << endl
         << " " << P1 << endl
         << endl;

    system("pause");
    system("cls");

    /***********************************************************************************
        Now let's take a quick look at our first images from the left camera and
        the right camera.
    ***********************************************************************************/
    Mat first_img_left = imread("../dataset/sequences/00/image_0/000000.png", IMREAD_GRAYSCALE);
    Mat first_img_right = imread("../dataset/sequences/00/image_1/000000.png", IMREAD_GRAYSCALE);
    imshow("first left image", first_img_left);
    imshow("first right image", first_img_right);

    waitKey();
    destroyWindow("first left image");
    destroyWindow("first right image");

    /*
     *	Part II - Stereo Depth & Visual Odometry
     */

    /***********************************************************************************
        Compute disparity using StereoBM

        Using Functions:
            computeLeftDisparityMap()

    ***********************************************************************************/
    Mat disp = method.computeLeftDisparityMap(first_img_left, first_img_right, BM, false);

    Mat color_disp;
    disp.convertTo(color_disp, first_img_left.type());
    applyColorMap(color_disp, color_disp, COLORMAP_VIRIDIS);
    imshow("first images disparity map using StereoBM", color_disp);

    waitKey();

    /***********************************************************************************
        Now to compare this to StereoSGBM
    ***********************************************************************************/
    disp = method.computeLeftDisparityMap(first_img_left, first_img_right, SGBM, false);

    disp.convertTo(color_disp, first_img_left.type());
    applyColorMap(color_disp, color_disp, COLORMAP_VIRIDIS);
    imshow("first images disparity map using StereoSGBM", color_disp);

    waitKey();
    destroyWindow("first images disparity map using StereoBM");
    destroyWindow("first images disparity map using StereoSGBM");
    system("cls");

    /***********************************************************************************
        We can see that StereoSGBM takes around 3x as long, but produces a much more
        contiguous disparity map, with less gaps in information

        We can see that there is a gap of the left side of the image where the right
        camera did not have matching information. This means that we should apply a
        mask when looking for features to match from one frame to the next so that we
        can use features which fall in the area of the picture for which we have depth
        information.
    ***********************************************************************************/

    /***********************************************************************************
        Now we want to get depth map.
        It will require the disparity map of the desire pair of images, and the
        intrinsic matrix, translation vector, and translation vector of the pair of
        cameras.
        We just got the disparity map of the first pair of images, so we are going to
        get the other 3 requirements by decomposing the projection matrices.

        Using Functions:
            decompose_Projection_Matrix()

    ***********************************************************************************/
    Mat k_left, r_left, t_left;
    Mat k_right, r_right, t_right;
    method.decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
    method.decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);
    cout << "kl:" << k_left << endl;
    cout << "kr:" << k_right << endl;

    /***********************************************************************************
        Now we can generate the depth map.

        Using Functions:
            calc_depth_map()

    ***********************************************************************************/
    Mat depth = method.calc_depth_map(disp, k_left, t_left, t_right, true);

    Mat color_depth;
    depth.convertTo(color_depth, first_img_left.type());
    applyColorMap(color_depth, color_depth, COLORMAP_VIRIDIS);
    imshow("depth map", color_depth);

    /***********************************************************************************
        Let's see what the depth is in the yellow band to the left

    ***********************************************************************************/
    printf("\n\ndepth[0, 0] = %f\n", depth.at<float>(0, 0));

    /***********************************************************************************
        Let's see if this is the same as the maximum estimated depth

    ***********************************************************************************/
    float depth_max = 0.0;
    for (int i = 0; i < depth.rows; i++)
        for (int j = 0; j < depth.cols; j++)
            if (depth.at<float>(i, j) > depth_max)
                depth_max = depth.at<float>(i, j);
    printf("depth_max = %f\n", depth_max);

    /***********************************************************************************
        We want to find the width of the yellow band, in order to create a mask to
        prevent the feature detector from searching in a useless area for features
        on every frame, which would waste time.

    ***********************************************************************************/
    for (int i = 0; i < depth.cols; i++)
        if (depth.at<float>(4, i) < depth_max)
        {
            printf("First non-max value at index %d\n", i);
            break;
        }

    waitKey();
    destroyWindow("depth map");

    /***********************************************************************************
        We can constuct a mask using this information like so

    ***********************************************************************************/
    Mat mask = Mat::zeros(depth.rows, depth.cols, CV_8U);
    mask(Rect(96, 0, depth.cols - 96, depth.rows)) = 255;

    Mat color_mask;
    applyColorMap(mask, color_mask, COLORMAP_VIRIDIS);
    imshow("MASK", color_mask);
    waitKey();
    destroyWindow("MASK");

    /***********************************************************************************
        Ok. Now we make an all-inclusive function to get the depth from an incoming
        set of stereo images

        <-- see stereo_2_depth() in Method.cpp
    ***********************************************************************************/

    /***********************************************************************************
        Look at matched points using sgbm matcher.

    ***********************************************************************************/
    /*depth = method.stereo_2_depth(first_img_left,
        first_img_right,
        P0,
        P1,
        SGBM,
        false,
        true);

    system("cls");*/

    /***********************************************************************************
        First, we extract features of 1st and 2nd images of the left camera.
        By input the images and mask, the extract_features() function will give us kp,
        which is an array of keypoints (feature points), and the descriptors of these
        points.

        vector	KeyPoint				 Mat[0]	 128 data to describe features

        kp[0]	(x0, y0)		-->		 des[0]  (data0, data1, ..., data127)
        kp[1]	(x1, y1)		-->		 des[1]  (data0, data1, ..., data127)
        .									.
        .									.
        .									.
        kp[n]	(xn, yn)		-->		 des[n]  (data0, data1, ..., data127)

    ***********************************************************************************/
    Mat des0, des1;
    vector<KeyPoint> kp0, kp1;
    Mat second_img_left = imread("../dataset/sequences/00/image_0/000001.png", IMREAD_GRAYSCALE);
    des0 = method.extract_features(first_img_left, Orb, mask, &kp0);
    des1 = method.extract_features(second_img_left, Orb, mask, &kp1);

    Mat image_matches;
    drawKeypoints(first_img_left, kp0, image_matches);
    imshow("image matches", image_matches);

    drawKeypoints(second_img_left, kp1, image_matches);
    imshow("image matches2", image_matches);
    waitKey();
    destroyWindow("image matches");
    destroyWindow("image matches2");
    system("cls");
    /***********************************************************************************
        We got the keypoints and their descriptors of the 1st and 2nd image of the
        left camera.
        Next, we'll match those keypoints by their desciptors using knn. We set k=2
        here, so for each pair kp0, there will be 2 kp1 that are the most similar and
        second most similar, according to their descriptors.

        Conclusion: each kp0 has kp1_1, kp1_2, after matching feature

        Each matches[i] corresponses to kp0[i], where matches[i][0] = kp1_1 and
        matches[i][1] = kp1_2

    ***********************************************************************************/
    vector<DMatch> matches = method.match_features(des0, des1, BF, Orb, false, 2, 0);

    printf("Number of matches before filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    system("cls");

    /***********************************************************************************
        The reason we take 2 matching points for each kp0 instead of 1, is that if kp0
        is similar to kp1_1 and kp1_2, then we can assume that kp0 is not distinct
        enough.
        So, we filter kp0 by comparing kp1_2 and kp1_1*dist_threshold. If kp1_2 <=
        kp1_1*dist_threshold, then we say kp0 is distinct enough to be a match.

    ***********************************************************************************/
    matches = method.match_features(des0, des1, BF, Orb, false, 2, 0.6);
    // vector<DMatch> matches = method.filter_matches_distance(matches_unfilt, 0.6);
    printf("Number of matches after filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    system("cls");

    /***********************************************************************************
        Now to see the difference with 'bm' matcher

    ***********************************************************************************/
    depth = method.stereo_2_depth(first_img_left,
                                  first_img_right,
                                  P0,
                                  P1,
                                  BM,
                                  false,
                                  true);

    system("cls");

    des0 = method.extract_features(first_img_left, Sift, mask, &kp0);
    des1 = method.extract_features(second_img_left, Sift, mask, &kp1);
    matches = method.match_features(des0, des1, BF, Sift, true, 2, 0);
    printf("Number of matches before filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    // matches = method.filter_matches_distance(matches, 0.3);
    matches = method.match_features(des0, des1, BF, Sift, true, 2, 0.3);
    printf("Number of matches after filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    /***********************************************************************************
        We can see that the 'bm' matcher is around 5x faster than the 'sgbm' matcher,
        and produced the same number of matches.Since speed is essential, we will use
        the 'bm' matcher

    ***********************************************************************************/

    /***********************************************************************************
        Now to see the difference with ORB descriptor

    ***********************************************************************************/
    /*depth = method.stereo_2_depth(first_img_left,
        first_img_right,
        P0,
        P1,
        BM,
        false,
        true);

    system("cls");*/

    des0 = method.extract_features(first_img_left, Orb, mask, &kp0);
    des1 = method.extract_features(second_img_left, Orb, mask, &kp1);
    matches = method.match_features(des0, des1, BF, Orb, true, 2, 0);
    printf("Number of matches before filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    // matches = method.filter_matches_distance(matches, 0.8);
    matches = method.match_features(des0, des1, BF, Orb, true, 2, 0.3);
    printf("Number of matches after filtering : %d\n", matches.size());
    method.visualize_matches(first_img_left, kp0, second_img_left, kp1, matches);

    Mat rmat, tvec, image1_points, image2_points, trans_mat;
    method.estimate_motion(matches, kp0, kp1, k_left, depth, 3000, rmat, tvec, image1_points, image2_points);

    Mat I4 = Mat::eye(4, 4, CV_64F);
    hconcat(rmat, tvec, trans_mat);
    vconcat(trans_mat, I4.row(3), trans_mat);

    cout << "t_tot:\n"
         << trans_mat << endl;
    cout << "\n\nt_tot_inv:\n"
         << trans_mat.inv() << endl;
}

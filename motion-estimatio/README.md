# **Motion Estimation**
Estimating the relative motion between camera poses using depth map and matched keypoints. This kernel is using **RANSAC-EPnP** (RANdom SAmple Consensus, Efficient Perspective-n-Point) and **Iterative PnP** (Direct Linear Transform).


## **Design Flow**
- [Algorithms](#algorithms)    
- [Pure C/C++ code](./c-src)
- [HLS code](./hls-src/)
- [Reference](#reference)

## **Algorithms**
The brief introduction can be found in the [slide](./doc/motion_estimation.pdf). You may also check out the [video](https://youtu.be/1_DqewUjm7Q).

- ### What we have
  ***depth map*** from Stereo Matching

  matched 2D ***keypoints*** of 1st and 2nd left images

  ***camera intrinsic matrix*** 

  Now we are looking for Rotation matrix ***rmat*** and translation vector ***tvec*** to estimate the relative motion between 1st and 2nd images. 

  ```cpp
  void estimate(Matrix &match, Matrix &kp0, Matrix &kp1, Matrix &k, Matrix &depth, Matrix &rmat, Matrix &tvec);
  ```

- ### Projection
  Before doing the math part, we need to modify the input to match the requirement. Because we're using PnP, which solves relative motion cv problem by 2D-3D points. As a result, we project 1st left image's keypoints to 3D point through depth map and camera matrix. Also we align 2 sets of keypoints through matched index.

<p align="center">
  <img src="./img/Motion%20Estimation%20block%20diagram.png" />
  <img src="./img/EPnP%20block%20diagram.png" />
</p>

## **Pure C/C++ code**
Extracting and modifying source code from OpenCV, the pure C/C++ code implement most of the function [solvePnPRansac](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html). It's runnable for CPU, but may not be able to synthesis due to dynamic allocated memory and some other reasons.

## **HLS code**
## [underbaseline](./hls-src/c-synth_underbaseline)
The codes are synthesizable are and able to do co-simulation, while no optimization is done. In fact, these codes are outcome when taking co-simulation as priority. The utilization may be too high to implement on the board. 

## **Reference**
- [PnP求解--EPnP | EpsilonJohn's Blog](http://epsilonjohn.club/2020/02/26/SLAM%E4%BB%A3%E7%A0%81%E8%AF%BE%E7%A8%8B/PnP%E6%B1%82%E8%A7%A3-EPnP/#epnp)
- [深入EPnP算法_JesseChen79的博客-CSDN博客_epnp算法](https://blog.csdn.net/jessecw79/article/details/82945918)
- [PnP 算法简介 & 代码解析 - 柴政](https://youtu.be/EX8Y9kB1sSw)
- [Newton's method for solving nonlinear systems of Algebraic equations](https://youtu.be/zPDp_ewoyhM)

- [Singular Value Decomposition (SVD) - 2022.2 English - Xilinx](https://docs.xilinx.com/r/en-US/Vitis_Libraries/quantitative_finance/guide_L1/SVD/SVD.html)
# **Motion Estimation**
Estimating the relative motion between camera poses using depth map and matched keypoints. This kernel is using **RANSAC-EPnP** (RANdom SAmple Consensus, Efficient Perspective-n-Point) and **Iterative PnP** (Direct Linear Transform).

## **Design Flow**
- [Algorithms](#algorithms)    
- [Pure C/C++ code](./c-src)
- [HLS code](./c-src)

## **Algorithms**
The brief introduction is in the [slide](./doc/motion_estimation.pdf). You may also check out the [video](https://youtu.be/1_DqewUjm7Q).


![block diagram](./img/Motion%20Estimation%20block%20diagram.png)

## **Pure C/C++ code**
Extracting and modifying source code from OpenCV, the pure C/C++ code implement most of the function [RANSAC_PNP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html). It's runnable for CPU, but may not be able to synthesis due to dynamic allocated memory and some other reasons.

## **HLS code**
## underbaseline
# FPGA Implementation of Visual Odometry using High-Level Synthesis
[![](https://img.shields.io/badge/vo--hls-report-brightgreen)](https://implementation.ee.nthu.edu.tw/competition/groups/d654e3e1-c800-43e4-8583-01de78e7f9eb/attachments/summary?download=0)

This repository contains the related work for the NTHU 11110EE 390000 / 11120EE 391000 Special Topic on Implementation (I)/(II) courses, specifically from [Team A288](https://implementation.ee.nthu.edu.tw/competition/groups/111/2). 

The contributors are as follows:

- [林奕杰](https://github.com/yichiehqq)
- [李承澔](https://github.com/Charlee0207)
- [鄧文瑜](https://github.com/kevinteng9023)
- [林妤諠](https://github.com/Yuhsuanlinn)
- [郭朝恩](https://github.com/ShinjuGoenji)


Most research and implementations in the field of VO focus on specific algorithms such as Stereo Matching, Feature Extraction, Feature Tracking (Matching), and Motion Estimation. In this project, we propose that these four stages of the VO algorithm can be implemented and executed on the Xilinx Alveo™ U50 FPGA board, thereby exploring the possibility of accelerating these algorithms in comparison to traditional CPU-based approaches.

## Abstract Development Flow and System Architecture

To begin, we selected a Python OpenCV tutorial on VO as a template and re-implemented it in C/C++ form with HLS synthesis accessibility. We removed all external library dependencies and eliminated CPU architecture-specific coding styles such as dynamic memory allocation and double-level pointers.

![](./doc/img/algorithm%20flow.gif)
> The VO algorithm is divided into four sub-algorithms: Stereo Matching, Feature Extraction, Feature Tracking, and Motion Estimation. 


![](./doc/img/system_arch.png)
> Each sub-algorithm corresponds to a kernel.

In order to properly verify the functionality of each kernel, we included four compile modes to run the VO program. In each compile mode, the corresponding kernel function runs on the programmable logic (PL) side of the FPGA, while the other three functions run on the processing system (PS) side (the host). This ensures that each individual kernel behaves correctly and allows us to compare the execution time between the FPGA and the CPU.

## Toolchain and Prerequisites
The project has been tested in the following environment:
- CPU: 11th Gen Intel(R) Core(TM) i7-11700 @ 2.50GHz
- RAM: 49112592kB DDR4
- OS: Ubuntu 20.04.4 LTS
- FPGA: Xilinx Alveo™ U50 FPGA
- Vitis Suite Version: 2022.1
- OpenCV Version: 4.4.0

Prerequisites:
- Ubuntu 20.04 (or a higher version certified with Vitis)
- Xilinx Vitis Suite 2022.1
- Xilinx® Runtime (XRT)
- CMake 3.5+
- OpenCV-4.4.0 x86 library
- libOpenCL.so
- libpng library (optional)

Please follow the detailed [tutorial](https://hackmd.io/@PVeFLV0TSLusVkTPYj7DuQ/S15_qtAio) for setup instructions.

## Directory Structure
   ```
    ${ProjectFolder}
      |- ${HostFolder}
      |   |- src
      |   |   |- C_FeatureExtraction  
      |   |   |- C_FeatureTracking
      |   |   |- C_MotionEstimation
      |   |   |- C_StereoMatching
      |   |   |- K_FeatureExtraction
      |   |   |- K_FeatureTracking
      |   |   |- K_MotionEstimation
      |   \   \- K_StereoMatching
      |  
      |- output  
      |   |- C_FeatureExtraction  
      |   |- C_FeatureTracking
      |   |- C_MotionEstimation
      |   |- C_StereoMatching
      |   |- K_FeatureExtraction
      |   |- K_FeatureTracking
      |   |- K_MotionEstimation
      |   \- K_StereoMatching
      |- dataset
      |   |- poses
      \   \- sequences 
   ```
   Please download the [dataset.zip](https://drive.google.com/file/d/10q1iml4rOL9GB1Ew3EcNF0lm_c7B5Nf_/view?usp=drivesdk) from drive and unzip.

## Quick Start
### Prepare the Environment

### Build Bitstream from Each Kernel Function

### Build Whole System using Bitstream

### Run

### Flags



## Result


## Appendix


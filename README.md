# FPGA Implementation of Visual Odometry using High-Level Synthesis
[![](https://img.shields.io/badge/vo--hls-report-brightgreen)](https://implementation.ee.nthu.edu.tw/competition/groups/d654e3e1-c800-43e4-8583-01de78e7f9eb/attachments/summary?download=0)

![](./doc/img/algorithm%20flow.gif)

The goal of this project is to utilize High-Level Synthesis (HLS) technology for hardware acceleration to increase development efficiency and simplify the design process. Compared to traditional hardware description languages such as Verilog and VHDL, HLS can quickly translate high-level programming languages such as C, C++, SystemC into RTL, and generate higher-performance hardware architectures through different optimizations, allowing designers to develop complex hardware systems more quickly. \
In this project, we use [Xilinx Vitis](https://www.xilinx.com/products/design-tools/vitis/vitis-platform.html) development system to rewrite algorithms in OpenCV and implement the [Visual Odometry](https://github.com/FoamoftheSea/KITTI_visual_odometry.git) (VO) algorithm using HLS technology. We will use knowledge of computer vision, system design, hardware design, HLS development process, and HLS technology to achieve efficient real-time computation of the VO accelerator on FPGA.

This repository contains the related work for the NTHU 11110EE 390000 / 11120EE 391000 Special Topic on Implementation (I)/(II) courses, specifically from Team A288. The contributors are as follows:

- 林奕杰 
- 李承澔
- 鄧文瑜
- 林妤諠
- 郭朝恩

Most research and implementations in the field of VO focus on specific algorithms such as Stereo Matching, Feature Extraction, Feature Tracking (Matching), and Motion Estimation. In this project, we propose that these four stages of the VO algorithm can be implemented and executed on the Xilinx Alveo™ U50 FPGA board, thereby exploring the possibility of accelerating these algorithms in comparison to traditional CPU-based approaches.

## Abstract Development Flow and System Architecture

To begin, we selected a Python OpenCV tutorial on VO as a template and re-implemented it in C/C++ form with HLS synthesis accessibility. We removed all external library dependencies and eliminated CPU architecture-specific coding styles such as dynamic memory allocation and double-level pointers.

The VO algorithm is divided into four sub-algorithms: Stereo Matching, Feature Extraction, Feature Tracking, and Motion Estimation. Each sub-algorithm corresponds to a kernel.

In order to properly verify the functionality of each kernel, we included four compile modes to run the VO program. In each compile mode, the corresponding kernel function runs on the programmable logic (PL) side of the FPGA, while the other three functions run on the processing system (PS) side (the host). This ensures that each individual kernel behaves correctly and allows us to compare the execution time between the FPGA and the CPU.



## Directory
```
├── SGBM                    # stereo matching kernel
├── feature-extraction      # feature extraction kernel
├── feature-matching        # feature matching kernel
├── motion-estimation       # motion estimation kernel
├── host                    # host program
└── doc                     # reference files
```

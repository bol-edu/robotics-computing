# Robotics Computing
[![](https://img.shields.io/badge/vo--hls-report-brightgreen)](https://implementation.ee.nthu.edu.tw/competition/groups/d654e3e1-c800-43e4-8583-01de78e7f9eb/attachments/summary?download=0)

![](./doc/img/algorithm%20flow.gif)

The goal of this project is to utilize High-Level Synthesis (HLS) technology for hardware acceleration to increase development efficiency and simplify the design process. Compared to traditional hardware description languages such as Verilog and VHDL, HLS can quickly translate high-level programming languages such as C, C++, SystemC into RTL, and generate higher-performance hardware architectures through different optimizations, allowing designers to develop complex hardware systems more quickly. \
In this project, we use [Xilinx Vitis](https://www.xilinx.com/products/design-tools/vitis/vitis-platform.html) development system to rewrite algorithms in OpenCV and implement the [Visual Odometry](https://github.com/FoamoftheSea/KITTI_visual_odometry.git) (VO) algorithm using HLS technology. We will use knowledge of computer vision, system design, hardware design, HLS development process, and HLS technology to achieve efficient real-time computation of the VO accelerator on FPGA.

## Directory
```
├── SGBM                    # stereo matching kernel
├── feature-extraction      # feature extraction kernel
├── feature-matching        # feature matching kernel
├── motion-estimation       # motion estimation kernel
├── host                    # host program
└── doc                     # reference files
```
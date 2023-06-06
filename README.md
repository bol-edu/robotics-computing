# Robotics Computing
![](./doc/img/algorithm%20flow.gif)

The goal of this project is to utilize High-Level Synthesis (HLS) technology for hardware acceleration to increase development efficiency and simplify the design process. Compared to traditional hardware description languages such as Verilog and VHDL, HLS can quickly translate high-level programming languages such as C, C++, SystemC into RTL, and generate higher-performance hardware architectures through different optimizations, allowing designers to develop complex hardware systems more quickly. \
In this project, we use Xilinx [Vitis](https://www.xilinx.com/products/design-tools/vitis/vitis-platform.html) development system to rewrite algorithms in OpenCV and implement the [Visual Odometry](https://github.com/FoamoftheSea/KITTI_visual_odometry.git) (VO) algorithm using HLS technology. We will use knowledge of computer vision, system design, hardware design, HLS development process, and HLS technology to achieve efficient real-time computation of the VO accelerator on FPGA.\
本專題旨在運用高階合成 (High-Level Synthesis，HLS) 技術進行硬體加速，以提高開發效率並簡化設計流程。相對於傳統使用 Verilog、VHDL 等硬體描述語言進行開發，HLS 技術能夠將高階程式語言 (C、C++、SystemC) 快速轉譯為 RTL，並透過不同的優化產生更高效能的硬體架構，讓設計者能夠更快速地開發複雜的硬體系統。\
在此專題中，我們使用 Xilinx 的 [Vitis](https://www.xilinx.com/products/design-tools/vitis/vitis-platform.html) 開發系統，將 OpenCV 的演算法改寫，利用HLS 技術實現[視覺里程計](https://github.com/FoamoftheSea/KITTI_visual_odometry.git)（Visual Odometry，VO）演算法。我們將使用電腦視覺、系統設計、硬體設計、HLS 開發流程、HLS 技術等相關知識，以實現 VO 加速器於 FPGA 的高效即時運算。

## Visual Odometry Algorithm
![](https://i.imgur.com/nidbTA0.png)

Visual Odometry 的目標是藉由位移前後擷取的兩張影像，估算出位移量，其中位移量以旋轉矩陣 R、平移向量 t 表示。如上圖示，由影像轉換到位移量的過程會經過[深度估計](https://docs.opencv.org/3.4/dd/d53/tutorial_py_depthmap.html) (Stereo Matching)、[特徵點擷取](https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html) (Feature Extraction)、[特徵點配對](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) (Feature Matching)、以及[位移估測](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) (Motion Estimation) 四個步驟。\
在每次位移的估計中，首先需要輸入位移前左、右兩部相機擷取的圖片 (image left 0、image right 0)，配合相機參數 (P0、P1)，用左右視差計算出深度 (depth)。同時分別對左相機位移前後擷取的圖片(image left 0、image left 1) 提取特徵點 (kp0、kp1) 以及其描述子 (des0、des1)。
在提取特徵點後，給定閥值 (filter) 利用描述子配對特徵點 (matches)。
最後利用深度、配對的特徵點、相機參數，估計出旋轉矩陣 (rmat) 以及平移向量 (tvec)。
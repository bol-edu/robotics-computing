# Stereo Matching
SGBM 是「Semi-Global Block Matching」的縮寫，此演算法是一種用於立體視覺和深度估計的演算法，它可以在左右相機圖像之間進行視差計算(位移)。SGBM 是一種基於區塊匹配的演算法，在左右圖像中匹配特定大小的區塊，並計算每個區塊在另一個圖像中的對應區塊的相似度，以確定每個像素的視差值。且會考慮每個像素的整個代價（cost）圖，並將其與經過平滑處理的代價圖進行比較，以選擇最佳匹配 (WTA)。
    <img width="384" alt="image" src="https://github.com/bol-edu/robotics-computing/assets/99881755/c4eaf86f-6ba9-4b8d-9fb2-3d9dff1056ec">

- ### 輸入/輸出
    輸入 : 將左右兩張圖片以及相機的 k、t、r 參數讀進來。     
    輸出 : 經過計算後輸出深度。
- ### matching cost computation (匹配代價計算)預處理
    1. 輸入圖像經過 SobelX 處理后，計算 BT 代價。
    2. 輸入圖像直接計算 BT 代價值。
    3. 將上面兩步的代價值進行融合。
    4. 對上述步驟得到的代價值進行成塊處理，在 SGBM 演算法中，成塊計算就是就是對每個圖元的代價值用周圍鄰域代價值的總和來代替。
- ### cost aggregation (代價聚合) 
    <img width="478" alt="image" src="https://github.com/bol-edu/robotics-computing/assets/99881755/28a0837d-21b5-400c-8393-a4e6eafb9c6f">


    L為當前路徑累積的代價函數，P1、P2為圖元點與相鄰點視差存在較小和較大差異情況下的平滑懲罰(P1<P2)，第三項是為了消除各個方向路徑長度不同造成的影響。將所有r方向的匹配代價相加得到總的匹配代價(代價聚合)。
- ### disparity computation/optimization (視差計算或優化)
    1. sgbm演算法中關於視差的選擇，一般是直接選擇使得聚合代價最小的視差。
    2. 每個像素的視差選擇都只是簡單通過 WTA（Winner Takes All）決定的。
- ### disparity refinement (視差改良)後處理
    1. 利用參數來調整最後計算出的結果

## Parameters
   <img width="471" alt="image" src="https://github.com/bol-edu/robotics-computing/assets/99881755/f8483508-672a-46dc-9348-c79b68d3ce55">

## On Visual Studio
- ### Original C++ code
    從 OpenCV 中整理出與 SGBM 演算法相關的程式碼，並進一步進行簡化，但仍有 include 到 OpenCV library 的函式。
    
    [c-src](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/c-src)

- ### Pure (self-contained) C/C++ code
    透過改寫使其完全無引用 OpenCV library。
    
    [c-src_self-contain](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/c-src_self-contain)

## On Vitis_hls
   到 Vitis_HLS 的環境上運行，依序跑過以下三個階段並驗證正確性，且透過改寫程式碼並加上 pragma 使其符合 hls 的規則並使其可合成為硬體。
- ### C-simulation
    [hls-src](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/hls-src)

- ### Synthesis
    [hls-syn](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/hls-syn)

- ### Co-simulation
    [hls-cosim](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/hls-cosim)

## On Vitis IDE (FPGA)
   到 Vitis ide 環境中實際跑硬體合成、燒錄並產生出 bitstream (.xclbin file)。
- ### Optimization
    [hardware](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/hardware)
    
## development log
在以下資料夾的 ppt 中，有記錄了一些我在此次專題製作過程中遇到的問題及最後的解決辦法，另外，也有 SGBM 演算法的相關步驟流程介紹。
    
   [ppt](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/ppt)
    
## makefile
   [program](https://github.com/bol-edu/robotics-computing/tree/main/SGBM/program)
   
## Build Flow
    - build 指令
    make all TARGET=hw PLATFORM=xilinx_u50_gen3x16_xdma_5_202210_1 CXXFLAGS=-I/usr/local/include/opencv4




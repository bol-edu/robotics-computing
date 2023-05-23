# Feacture Extraction

## Algorithms

![resource limitation](https://i.imgur.com/D941eFh.png)
    
- ### 輸入／輸出
    輸入：要計算的image與給定的mask。
    輸出：偵測到的keypoints與計算出的descriptors。

- ### 創建尺度金字塔
    將原始圖片進行不同尺度的縮放，以確保特徵點檢測的尺度不變性，也就是在不同尺度下的檢測結果相同。
    我們採用8層尺度金字塔。
- ### 特徵點檢測
    採用 oriented FAST (oFAST) 的檢測方式，oFAST結合了FAST與Harris兩種算法。
  
  1. FAST算法檢測特徵點的方式為當一像素其周圍圓上有連續n個像素的灰度值大於或小於給定的臨界值，此像素即為特徵點。oFAST採用半徑為8的圓、連續9個像素，且臨界值為20。
  2. Harris算法為偵測圖像中的角點，角點是兩條邊緣的交點，因此檢測小圖像片段中灰度值的變化程度，即可判斷特徵點。
  3. 最後將每個特徵點加入角度，以確保特徵點的旋轉不變性，也就是在不同方向下的檢測結果相同。角度的計算為灰度質質心位置到中央與x軸所產生的夾角。
  
  
- ### 計算描述子(Descriptor)
  1. 將圖片做高斯模糊，使得每一鄰近像素之間的落差不會太大，以確保描述子計算的正確性。
  2. 以特徵點為中心，在31\*31的圓內選取256對點做比較，每一對點會得到1與0的結果，此256個bit即為描述子。
  
## C++ code 

使用到 OpenCV 函式庫，由 OpenCV 函式庫中整理出特徵點檢測演算法的程式碼。
* [c-src](https://github.com/bol-edu/robotics-computing/tree/main/feature-extraction/c-src)

## Pure C/C++ code

沒有使用到 OpenCV 函式庫，由 OpenCV 函式庫中整理出特徵點檢測演算法的程式碼，並不再使用到 OpenCV 函式庫即可在CPU中運算。
* [c-src_self-contain](https://github.com/bol-edu/robotics-computing/tree/main/feature-extraction/c-src_self-contain)

## HLS code

能夠合成，但是無優化。
* [src_hardware](https://github.com/bol-edu/robotics-computing/tree/main/feature-extraction/src_hardware)

## HLS code optimized

能夠合成，且過初步優化。
* [src_optimized](https://github.com/bol-edu/robotics-computing/tree/main/feature-extraction/src_optimized)
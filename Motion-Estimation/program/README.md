## What's in Here
```
.
├── testdata                 // 測試資料
├── host                     // estimate_motion host 程式碼
├── kernel                   // estimate_motion kernel 程式碼
├── Makefile                 // 用於 build vitis
├── README.md                // 說明文件
├── utils.mk                 // build 設定檔
├── args.mk                  // execute 引數設定檔
```

## Build the program
```
make all TARGET=hw
```
## Run the program
```
make run TARGET=hw
```

## Modify the Arguements
args.mk
```
PLATFORM_VENDOR := Xilinx
HW_PLATFORM := xilinx_u50_gen3x16_xdma_base_5
XCLBIN_NAME := estimate_motion.xclbin
INPUT_DIR := ./testdata
OUTPUT_DIR := ./output
THRESHOLD := 64
CONFIDENCE := 0.99
MAXITER := 1000
```

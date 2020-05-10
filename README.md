# Manual Structure From Motion
![Demo Gif](demo.gif)

## Overview
This tool enables user to load a sequence of images and do sparse 3D point cloud reconstruction.

## Key Features
1. Visualization of keypoint matches between pair of images.
2. Allow user to manually remove bad matches before reconstruction (which automated softwares do not offer).
3. Auto recommend next pair of images to match.
4. Bundle adjustment.
5. Visualization of reconstructed point cloud and camera poses.

## Build Environment
Best on Ubuntu 16.04 and above.

## Build Commands
1. Apt install dependencies
```
$> sudo apt update
$> sudo apt install build-essentials git cmake libvtk6-qt-dev libopencv-dev libceres-dev
```
2. Create /build folder under /MSFM
```
$> mkdir build
$> cd build
```
3. build
```
$> cmake ..
$> make
```

## Run
Under build folder, type
```
$> ./src/ManualSFM
```

## Run Tests
Under build folder, type
```
$> ./tst/ManualSFM_tst
```

# Manual Structure From Motion
![Demo Gif](demo.gif)

## Overview
This tool enables user to load a sequence of images and do sparse 3D point cloud reconstruction.

## Key Features
1. Visualization of keypoint matches between pair of images.
2. Allow user to manually remove bad matches before reconstruction (which automated softwares do not offer).
3. Auto recommend next pair of images to match.
4. Bundle adjustment.
5. Visualization of reconstructed point cloud and camera pose.

## Required Preinstalled Libraries
1. Cmake v3.0 and above (for compile)
2. OpenCV (for computer vision package)
3. Qt4 with VTK (for user interface)
4. blas and lapack (for speedy computation)

## Compile
1. cmake .
2. make

## Run
./ManualSFM

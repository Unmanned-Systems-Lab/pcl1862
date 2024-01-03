# Lidar Obstacle Detection

## Overview
This project provides a C++ library for processing point clouds obtained from lidar sensors. It includes functionalities such as filtering, segmentation, clustering, and bounding box estimation for detected objects in the point clouds.

## Features
- **Filtering:** Remove unnecessary points from the point cloud based on resolution and specified region of interest.
- **Segmentation:** Identify ground and obstacles in the point cloud using RANSAC plane segmentation.
- **Clustering:** Group points into clusters representing individual objects.
- **Bounding Box Estimation:** Calculate bounding boxes for each identified cluster in 3D space.

## Dependencies
- [Point Cloud Library (PCL)](https://pointclouds.org/): A comprehensive library for 3D point cloud processing.
- [Eigen](https://eigen.tuxfamily.org/): C++ template library for linear algebra.
- [OpenCV](https://opencv.org/): Open Source Computer Vision Library for image processing.
  
## Usage
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/lidar-obstacle-detection.git

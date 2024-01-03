# Lidar 障碍物检测

## 概述
这个 C++ 库提供了处理激光雷达点云的功能，包括过滤、分割、聚类和检测点云中物体的三维边界框。

## 特性
- **过滤：** 基于分辨率和感兴趣区域删除不必要的点。
- **分割：** 使用 RANSAC 平面分割法识别地面和障碍物。
- **聚类：** 将点云分组为表示单个物体的聚类。
- **边界框估计：** 为每个识别的聚类计算三维边界框。

## 依赖项
- [Point Cloud Library (PCL)](https://pointclouds.org/)：全面的三维点云处理库。
- [Eigen](https://eigen.tuxfamily.org/)：用于线性代数的 C++ 模板库。
- [OpenCV](https://opencv.org/)：用于图像处理的开源计算机视觉库。

## 使用方法
1. **构建和编译:**
   ```
   bash
   cd lidar-obstacle-detection
   mkdir build && cd build
   cmake ..
   make

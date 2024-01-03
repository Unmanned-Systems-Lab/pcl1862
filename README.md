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
1. **示例:**
   ```
   #include "processPointClouds.h"

   int main() {
       // 创建点云处理器
       lidar_obstacle_detection::ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;
   
       // 载入点云数据
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = ... // 在此处加载你的点云数据
   
       // 过滤点云
       cloud = pointCloudProcessor.FilterCloud(cloud, 0.1, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
   
       // 分割地面平面
       auto segmentedCloud = pointCloudProcessor.SegmentPlane(cloud, 100, 0.2);
   
       // 对障碍物进行聚类
       auto clusters = pointCloudProcessor.Clustering(segmentedCloud.second, 0.3, 10, 500);
   
       // 提取每个聚类的边界框
       for (const auto& cluster : clusters) {
           lidar_obstacle_detection::BoxQ boundingBox = pointCloudProcessor.BoundingBoxQ(cluster);
           // 根据需要处理或可视化边界框
       }
   
       return 0;
   }


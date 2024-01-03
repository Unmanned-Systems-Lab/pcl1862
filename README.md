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

## 函数详细描述

### `template<typename PointT> class ProcessPointClouds`

该类提供了激光雷达点云处理的各种功能。

#### 构造函数
- **ProcessPointClouds():** 默认构造函数。

#### 析构函数
- **~ProcessPointClouds():** 默认析构函数。

#### 成员函数

##### `void numPoints(PtCdtr<PointT> cloud);`
计算给定点云中的点数目。

##### `PtCdtr<PointT> FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);`
过滤点云，去除分辨率范围外的点。

##### `std::pair<PtCdtr<PointT>, PtCdtr<PointT>> SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud);`
将点云分为地面和障碍物两部分。

##### `std::pair<PtCdtr<PointT>, PtCdtr<PointT>> SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol);`
使用 RANSAC 算法分割地面平面。

##### `std::vector<PtCdtr<PointT>> Clustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);`
将点云聚类成多个物体。

##### `std::vector<PtCdtr<PointT>> DBSCAN_Clustering(PtCdtr<PointT> cloud, float clusterTolerance, float ClusterSize);`
使用 DBSCAN 算法进行聚类。

##### `BoxQ BoundingBox(PtCdtr<PointT> cluster);`
计算给定聚类的三维边界框。

##### `BoxQ BoundingBoxQ(PtCdtr<PointT> cluster);`
计算带有四元数旋转的三维边界框。

##### `BoxQ BoundingBox4dof(PtCdtr<PointT> cluster);`
计算带有四自由度的三维边界框。

##### `BoxQ BoundingBox2_5D(PtCdtr<PointT> cluster);`
计算带有 2.5 自由度的三维边界框。

##### `Box4dof BoundingBox_2_5D_cv(std::vector<cv::Point3f> points_objects);`
使用 OpenCV 计算带有 2.5 自由度的三维边界框。

##### `Eigen::Quaternionf eulerToQuaternion(float roll, float pitch, float yaw);`
将欧拉角转换为四元数。

##### `void euclideanClustering(const std::vector<cv::Point3f>& points, std::vector<std::vector<cv::Point3f>>& clusters, float distanceThreshold);`
使用欧氏距离进行点云聚类。

##### `PtCdtr<PointT> vectorToPoint3fToPointCloud(const std::vector<cv::Point3f>& input_points);`
将点云向量转换为 PCL 点云格式。

##### `std::vector<cv::Point3f> pointCloudToVectorPoint3f(PtCdtr<PointT> input_cloud);`
将 PCL 点云格式转换为点云向量。

##### `std::vector<BoxQ> generateMatFromPoints(const std::vector<cv::Point3f> &points, float resolution);`
从点云生成边界框矩阵。

##### `std::vector<Box4dof> refinedClusteringAndBoundingBox(std::vector<std::vector<cv::Point3f>> points_objectss);`
进行点云聚类和边界框计算，返回带有四自由度的边界框。

##### `std::vector<Box4dof> clusteringFromPoint3fVectorAndBoundingBox(std::vector<cv::Point3f> points_objects);`
从点云向量计算聚类并返回带有四自由度的边界框。

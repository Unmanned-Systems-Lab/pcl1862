#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann.hpp>

namespace lidar_obstacle_detection {

    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>

    class ProcessPointClouds {
    public:

        //constructor
        ProcessPointClouds();

        //deconstructor
        ~ProcessPointClouds();

        void numPoints(PtCdtr<PointT> cloud);

        PtCdtr<PointT>FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint,Eigen::Vector4f maxPoint);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol);

        std::vector<PtCdtr<PointT>>Clustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);

        std::vector<PtCdtr<PointT>>DBSCAN_Clustering(PtCdtr<PointT> cloud, float clusterTolerance, float ClusterSize);


        BoxQ BoundingBox(PtCdtr<PointT> cluster);

        BoxQ BoundingBoxQ(PtCdtr<PointT> cluster);

        BoxQ BoundingBox4dof(PtCdtr<PointT> cluster);

        BoxQ BoundingBox2_5D(PtCdtr<PointT> cluster);

        Box4dof BoundingBox_2_5D_cv(std::vector<cv::Point3f> points_objects);

        Eigen::Quaternionf eulerToQuaternion(float roll, float pitch, float yaw);

        void euclideanClustering(const std::vector<cv::Point3f>& points, std::vector<std::vector<cv::Point3f>>& clusters, float distanceThreshold);
        
        PtCdtr<PointT> vectorToPoint3fToPointCloud(const std::vector<cv::Point3f>& input_points);

        std::vector<cv::Point3f> pointCloudToVectorPoint3f(PtCdtr<PointT> input_cloud);

        std::vector<BoxQ> generateMatFromPoints(const std::vector<cv::Point3f> &points, float resolution);

        std::vector<Box4dof> refinedClusteringAndBoundingBox(std::vector<std::vector<cv::Point3f>> points_objectss);

         std::vector<Box4dof> clusteringFromPoint3fVectorAndBoundingBox(std::vector<cv::Point3f> points_objects);

    };
}
#endif /* PROCESSPOINTCLOUDS_H_ */
#include "processPointClouds.h"

using namespace lidar_obstacle_detection;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud) { std::cout << cloud->points.size() << std::endl; }


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint,
                                                       Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object: downsample the dataset using a leaf size of .2m
    pcl::VoxelGrid<PointT> vg;
    PtCdtr<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    PtCdtr<PointT> cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

//    return cloud;
    return cloudRegion;
}


template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    PtCdtr<PointT> obstCloud(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> planeCloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    // create extraction object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult(obstCloud,
                                                        planeCloud);
//    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
//	pcl::PointIndices::Ptr inliers; // Build on the stack
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // Build on the heap
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficient);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult = SeparateClouds(
            inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::DBSCAN_Clustering(PtCdtr<PointT> cloud, float clusterTolerance, float ClusterSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Create an instance of the EuclideanClusterExtraction class
    pcl::EuclideanClusterExtraction<PointT> ec;

    // Set the DBSCAN algorithm parameters
    //ec.setClusterTolerance(0.02);  // 2 cm
    //ec.setMinClusterSize(100);
    ec.setClusterTolerance(clusterTolerance);  // 2 cm
    ec.setMinClusterSize(ClusterSize);
    // Set the input point cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill the point cloud with data...
    std::vector<PtCdtr<PointT>> clusters;
    // Run the DBSCAN algorithm and obtain the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // For each cluster indice
    for (pcl::PointIndices getIndices: cluster_indices) {
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        // For each point indice in each cluster
        for (int index:getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::Clustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    //auto startTime = std::chrono::steady_clock::now();

    std::vector<PtCdtr<PointT>> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Build Kd-Tree Object
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // Input obstacle point cloud to create KD-tree
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices; // this is point cloud indice type
    pcl::EuclideanClusterExtraction<PointT> ec; // clustering object
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud); // feed point cloud
    ec.extract(clusterIndices); // get all clusters Indice

    // For each cluster indice
    for (pcl::PointIndices getIndices: clusterIndices) {
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        // For each point indice in each cluster
        for (int index:getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

    }

    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
    //           << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box4dof ProcessPointClouds<PointT>::BoundingBox_2_5D_cv(std::vector<cv::Point3f> points_objects) {
    
    std::vector<cv::Point2f> points2D;
    points2D.reserve(points_objects.size());

    for (const auto &point3D : points_objects) {
        points2D.emplace_back(point3D.x, point3D.y);
    }
    
    // 计算最小外接矩形
    cv::RotatedRect minAreaRect = cv::minAreaRect(points2D);

    // 初始化最大值和最小值为浮点数的最大值和最小值
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();;
    
    // 遍历points_objects中的点
    for (const auto& point : points_objects) {
        // 更新当前的最大值和最小值
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
       
    }
     
    // Initialize BoxQ object
    BoxQ boxQ;

    // Set bboxTransform using minAreaRect center and z = 0
    boxQ.bboxTransform = Eigen::Vector3f(minAreaRect.center.x, minAreaRect.center.y, (max_z+min_z)/2);

    // Set cube_length and cube_width using minAreaRect size
    boxQ.cube_length = minAreaRect.size.width;
    boxQ.cube_width = minAreaRect.size.height;
    boxQ.cube_height = max_z-min_z;

    // Convert minAreaRect angle from degrees to radians
    float angle_rad = minAreaRect.angle * M_PI / 180.0f;

    // Set bboxQuaternion with yaw angle from minAreaRect, roll and pitch angles are 0
    Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(angle_rad, Eigen::Vector3f::UnitZ());

    boxQ.bboxQuaternion = Eigen::Quaternionf(yawAngle * pitchAngle * rollAngle);

    Box4dof box4dof;
    box4dof.boxq=boxQ;
    box4dof.x=minAreaRect.center.x;
    box4dof.y=minAreaRect.center.y;
    box4dof.z=(max_z+min_z)/2;
    box4dof.w=minAreaRect.size.height;
    box4dof.l=minAreaRect.size.width;
    box4dof.h=max_z-min_z;
    box4dof.head=angle_rad;
    return box4dof ;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox2_5D(PtCdtr<PointT> cluster) {

    auto compare = [](const PointT& p1, const PointT& p2, const PointT& p0) {
        float angle1 = atan2(p1.y - p0.y, p1.x - p0.x);
        float angle2 = atan2(p2.y - p0.y, p2.x - p0.x);
        return angle1 < angle2;
    };

    auto counter_clockwise = [](const PointT& p1, const PointT& p2, const PointT& p3) {
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) > 0;
    };

    auto convex_hull = [&compare, &counter_clockwise](std::vector<PointT>& points) {
        size_t n = points.size(), k = 0;
        if (n <= 3) return points;

        std::vector<PointT> hull(2 * n);
        std::swap(points[0], *std::min_element(points.begin(), points.end(), [](const PointT& a, const PointT& b) { return a.y < b.y || (a.y == b.y && a.x < b.x); }));
        PointT p0 = points[0];
        std::sort(points.begin() + 1, points.end(), [&p0, &compare](const PointT& a, const PointT& b) { return compare(a, b, p0); });

        for (size_t i = 0; i < n; ++i)
        {
            while (k >= 2 && !counter_clockwise(hull[k - 2], hull[k - 1], points[i]))
                k--;
            hull[k++] = points[i];
        }

        hull.resize(k);
        return hull;
    };

    // Step 1: Set z values of cluster to 0
    std::vector<PointT> cluster_2d(cluster->points.begin(), cluster->points.end());
    for (auto& point : cluster_2d)
    {
        point.z = 0;
    }

    // Step 2: Compute 2D oriented bounding box with z values set to 0
    std::vector<PointT> hull = convex_hull(cluster_2d);

     // Compute PCA
    Eigen::MatrixXf data(2, hull.size());
    for (size_t i = 0; i < hull.size(); ++i)
    {
        data(0, i) = hull[i].x;
        data(1, i) = hull[i].y;
    }
    Eigen::Vector2f mean = data.rowwise().mean();
    data.colwise() -= mean;

    Eigen::Matrix2f cov = (data * data.transpose()) / (hull.size() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);

    // 计算旋转矩阵
    // Eigen::Matrix3f bbox_rotation = Eigen::Matrix3f::Identity(3, 3);
    // bbox_rotation.col(0) = eig.eigenvectors().real().col(0).homogeneous();
    // bbox_rotation.col(1) = eig.eigenvectors().real().col(1).homogeneous();
    // 计算旋转矩阵
    Eigen::Matrix3f bbox_rotation = Eigen::Matrix3f::Identity(3, 3);
    float yaw = std::atan2(eig.eigenvectors().real().col(0)(1), eig.eigenvectors().real().col(0)(0));
    bbox_rotation(0, 0) = std::cos(yaw);
    bbox_rotation(1, 0) = -std::sin(yaw);
    bbox_rotation(0, 1) = std::sin(yaw);
    bbox_rotation(1, 1) = std::cos(yaw);

     // 计算hull在新坐标系下的坐标
    std::vector<pcl::PointXYZI> rotated_hull;
    for (const auto& point : hull)
    {
        Eigen::Vector3f point_in(point.x, point.y, 0);
        Eigen::Vector3f rotated_point = bbox_rotation.transpose() * (point_in - Eigen::Vector3f(mean.x(), mean.y(), 0));
        pcl::PointXYZI new_point;
        new_point.x = rotated_point.x();
        new_point.y = rotated_point.y();
        new_point.z = rotated_point.z();
        rotated_hull.push_back(new_point);
    }

    // 计算包围盒长宽
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min();

    for (const auto& point : rotated_hull)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    float cube_length = max_x - min_x;
    float cube_width = max_y - min_y;

    // Step 3: 计算原始的cluster的z轴的最大值和最小值作为包围盒的高度
    float cube_height = 0.0;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();

    for (const auto& point : cluster->points)
    {
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    cube_height = max_z - min_z;

    // Step 4: 返回BoxQ各个值
    BoxQ box;
    box.bboxTransform = Eigen::Vector3f(mean.x(), mean.y(), (max_z + min_z) / 2);
    Eigen::Quaternionf quaternion = eulerToQuaternion(0, 0, bbox_rotation(2));
    box.bboxQuaternion = Eigen::Quaternionf(bbox_rotation);
    box.cube_length = cube_length;
    box.cube_width = cube_width;
    box.cube_height = cube_height;

    return box;
}
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox4dof(PtCdtr<PointT> cluster) {

   

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cluster, *cloud);

    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].z = 0;
    }

    // 计算当前簇的协方差矩阵
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

  
    // 获取OBB盒子
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    // 获取质心
    feature_extractor.getMassCenter (mass_center);

   

    // 添加OBB包容盒
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    // position：中心位置
    // quat：旋转矩阵
    // max_point_OBB.x - min_point_OBB.x  宽度
    // max_point_OBB.y - min_point_OBB.y  高度
    // max_point_OBB.z - min_point_OBB.z  深度
    //viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
    // Find bounding box for one of the clusters
    
    // Box4dof box;
    // box.x = position_OBB.x;
    // box.y = position_OBB.y;
    // box.z = position_OBB.z;
    // box.w = max_point_OBB.y - min_point_OBB.y;
    // box.h = max_point_OBB.z - min_point_OBB.z;
    // box.l = max_point_OBB.x - min_point_OBB.x;
    BoxQ boxq;
    boxq.bboxTransform = position;
    boxq.bboxQuaternion = quat;
    boxq.cube_length = max_point_OBB.x - min_point_OBB.x ;
    boxq.cube_width = max_point_OBB.y - min_point_OBB.y;
    boxq.cube_height = max_point_OBB.z - min_point_OBB.z;

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    boxq.cube_height=maxPoint.z-minPoint.z;
    //box.z = (maxPoint.z-minPoint.z)/2+minPoint.z;
    boxq.bboxTransform[2]=(maxPoint.z-minPoint.z)/2+minPoint.z;
    return boxq;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox(PtCdtr<PointT> cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    BoxQ bq;
    bq.bboxTransform[0]=(maxPoint.x+minPoint.x)/2;
    bq.bboxTransform[1]=(maxPoint.y+minPoint.y)/2;
    bq.bboxTransform[2]=(maxPoint.z+minPoint.z)/2;;

    Eigen::Quaternionf quaternion = eulerToQuaternion(0.0, 0.0, 0.0);
    bq.cube_height= maxPoint.z-minPoint.z;
    bq.cube_length= maxPoint.x-minPoint.x;
    bq.cube_width = maxPoint.y-minPoint.y;
    bq.bboxQuaternion = quaternion;
    return bq;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(PtCdtr<PointT> cluster) {


 // create a point cloud with type pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    // convert point cloud types
    pcl::copyPointCloud(*cluster, *cloud_out);
    // 计算当前簇的协方差矩阵
    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud (cluster);
    feature_extractor.compute ();

    PointT min_point_OBB;
    PointT max_point_OBB;
    PointT position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

  
    // 获取OBB盒子
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    // 获取质心
    feature_extractor.getMassCenter (mass_center);

   

    // 添加OBB包容盒
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    // position：中心位置
    // quat：旋转矩阵
    // max_point_OBB.x - min_point_OBB.x  宽度
    // max_point_OBB.y - min_point_OBB.y  高度
    // max_point_OBB.z - min_point_OBB.z  深度
    //viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    BoxQ box;
    box.bboxTransform = position;
    box.bboxQuaternion = quat;
    box.cube_length = max_point_OBB.x - min_point_OBB.x ;
    box.cube_width = max_point_OBB.y - min_point_OBB.y;
    box.cube_height = max_point_OBB.z - min_point_OBB.z;

    return box;
}

template<typename PointT>
Eigen::Quaternionf ProcessPointClouds<PointT>::eulerToQuaternion(float roll, float pitch, float yaw) {
    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());

    // Combine the rotations.
    Eigen::Quaternionf quaternion = yaw_angle * pitch_angle * roll_angle;

    return quaternion;
}
template<typename PointT>
void ProcessPointClouds<PointT>::euclideanClustering(const std::vector<cv::Point3f>& points, std::vector<std::vector<cv::Point3f>>& clusters, float distanceThreshold) {
     // Convert vector<Point3f> to cv::Mat
    cv::Mat pointsMat(static_cast<int>(points.size()), 3, CV_32F);
    for (size_t i = 0; i < points.size(); ++i) {
        pointsMat.at<float>(static_cast<int>(i), 0) = points[i].x;
        pointsMat.at<float>(static_cast<int>(i), 1) = points[i].y;
        pointsMat.at<float>(static_cast<int>(i), 2) = points[i].z;
    }

    // 使用OpenCV FlannBasedMatcher进行快速搜索
    cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index kdtree(pointsMat, indexParams);

    std::vector<bool> processed(points.size(), false);
    for (size_t i = 0; i < points.size(); ++i) {
        if (processed[i]) continue;

        std::vector<cv::Point3f> cluster;
        std::vector<int> searchQueue = { static_cast<int>(i) };

        for (size_t j = 0; j < searchQueue.size(); ++j) {
            int pointIdx = searchQueue[j];
            if (processed[pointIdx]) continue;

            processed[pointIdx] = true;
            cluster.push_back(points[pointIdx]);

            std::vector<float> query = { points[pointIdx].x, points[pointIdx].y, points[pointIdx].z };
            std::vector<int> indices;
            std::vector<float> distances;
            kdtree.radiusSearch(query, indices, distances, distanceThreshold * distanceThreshold, std::numeric_limits<int>::max(), cv::flann::SearchParams());

            for (int neighborIdx : indices) {
                if (!processed[neighborIdx]) {
                    searchQueue.push_back(neighborIdx);
                }
            }
        }

        clusters.push_back(cluster);
    }
}

template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::vectorToPoint3fToPointCloud(const std::vector<cv::Point3f>& input_points)
{
    typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    for (const auto& point : input_points)
    {
        PointT pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        output_cloud->points.push_back(pcl_point);
    }

    output_cloud->width = static_cast<uint32_t>(output_cloud->points.size());
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    return output_cloud;
}

template<typename PointT>
std::vector<cv::Point3f> ProcessPointClouds<PointT>::pointCloudToVectorPoint3f(PtCdtr<PointT> input_cloud)
{
    std::vector<cv::Point3f> output_points;

    for (const auto& point : input_cloud->points)
    {
        cv::Point3f cv_point(point.x, point.y, point.z);
        output_points.push_back(cv_point);
    }

    return output_points;
}

template<typename PointT>
std::vector<BoxQ> ProcessPointClouds<PointT>::generateMatFromPoints(const std::vector<cv::Point3f> &points, float resolution) {
    float x_min = FLT_MAX, x_max = FLT_MIN, y_min = FLT_MAX, y_max = FLT_MIN;

    for (const auto &point : points) {
        x_min = std::min(x_min, point.x);
        x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, point.y);
        y_max = std::max(y_max, point.y);
    }
    std::vector<BoxQ> boxQs;
    if(y_max-y_min>0.2 && x_max-x_min>0.2)
    {
    x_min = x_min-1.0;
    x_max = x_max+1.0;
    y_min = y_min-1.0;
    y_max = y_max+1.0;
    
    
        int width = static_cast<int>(( y_max - y_min) / resolution);
        int height = static_cast<int>((x_max - x_min ) / resolution);
        std::cout<<width<<" "<<height<<std::endl;
        cv::Mat img=cv::Mat::zeros(height,width, CV_8UC1);
        
        for (const auto &point : points) {
            int x_index = static_cast<int>( (point.y - y_min)/ resolution);
            int y_index = static_cast<int>((point.x - x_min) / resolution);
            img.at<uchar>( x_index,y_index) = 255;
        }

        // cv::namedWindow("Image", cv::WINDOW_NORMAL);
        // cv::imshow("Image", img);
        // cv::waitKey(0);
        //定义结构元素（卷积核）
        int kernelSize = 0.2/resolution;
        //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
        // 进行闭操作
        cv::Mat dst;
        cv::morphologyEx(img, dst, cv::MORPH_CLOSE, kernel);

        // cv::namedWindow("Image", cv::WINDOW_NORMAL);
        // cv::imshow("Image", dst);
        // cv::waitKey(0);
        
        //进行开操作
        cv::Mat dst1;
        kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
        cv::morphologyEx(dst, dst1, cv::MORPH_OPEN, kernel);

        // cv::imshow("Image", dst1);
        // cv::waitKey(0);

        //3. 查找轮廓
        //连通域分析
        cv::Mat labels, stats, centroids;
        int nLabels = connectedComponentsWithStats(dst1, labels, stats, centroids);
        
        //计算最小外接矩形
        //Initialize BoxQ object
        if(nLabels - 1>1) 
        {
        BoxQ boxQ;
        for (int i = 1; i < nLabels; i++)
        {
            // 提取连通域
            cv::Mat blob = (labels == i);

            // 计算最小外接矩形
            std::vector<cv::Point> points;
            cv::findNonZero(blob, points);
            cv::RotatedRect minAreaRect = cv::minAreaRect(points);
            // Set bboxTransform using minAreaRect center and z = 0
            boxQ.bboxTransform = Eigen::Vector3f(minAreaRect.center.x*resolution+x_min+1.0, minAreaRect.center.y*resolution+y_min+1.0,0);

            // Set cube_length and cube_width using minAreaRect size
            boxQ.cube_length = minAreaRect.size.width*resolution;
            boxQ.cube_width = minAreaRect.size.height*resolution;
            boxQ.cube_height = 0;

            // Convert minAreaRect angle from degrees to radians
            float angle_rad = minAreaRect.angle * M_PI / 180.0f;

            // Set bboxQuaternion with yaw angle from minAreaRect, roll and pitch angles are 0
            Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(0, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(angle_rad, Eigen::Vector3f::UnitZ());

            boxQ.bboxQuaternion = Eigen::Quaternionf(yawAngle * pitchAngle * rollAngle);

            boxQs.push_back(boxQ);

        }
        }

        return boxQs;
    }
    return boxQs;
}

template<typename PointT>
std::vector<Box4dof> ProcessPointClouds<PointT>::refinedClusteringAndBoundingBox(std::vector<std::vector<cv::Point3f>> points_objectss){
    std::vector<Box4dof> boxes;
    for (const auto& point : points_objectss){ 

        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds=vectorToPoint3fToPointCloud(point);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        PtCdtr<pcl::PointXYZ> cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(clouds);
        vg.setLeafSize(0.1, 0.1, 0.1);
        vg.filter(*cloudFiltered);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =Clustering(cloudFiltered,0.2,3,10000);
        
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
            std::vector<cv::Point3f> p3=pointCloudToVectorPoint3f(cluster);
            Box4dof box = BoundingBox_2_5D_cv(p3);
            boxes.push_back(box);
        }   
    }
    return boxes;
}

template<typename PointT>
std::vector<Box4dof> ProcessPointClouds<PointT>::clusteringFromPoint3fVectorAndBoundingBox(std::vector<cv::Point3f> points_objects){
    std::vector<Box4dof> boxes;

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds=vectorToPoint3fToPointCloud(points_objects);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    PtCdtr<pcl::PointXYZ> cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(clouds);
    vg.setLeafSize(0.2, 0.2, 0.2);
    vg.filter(*cloudFiltered);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =Clustering(cloudFiltered,0.4,3,10000);
        
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::vector<cv::Point3f> p3=pointCloudToVectorPoint3f(cluster);
        Box4dof box = BoundingBox_2_5D_cv(p3);
        boxes.push_back(box);
    }   

    return boxes;
}
template class ProcessPointClouds<pcl::PointXYZ>;
//template class ProcessPointClouds<pcl::PointXYZI>;
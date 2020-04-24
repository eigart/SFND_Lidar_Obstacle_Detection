// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    
    typename pcl::PointCloud<PointT>::Ptr cloud_voxels (new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloud_voxels);

    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(cloud_voxels);
    crop.filter(*cloud_cropped);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_cropped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud_cropped);
    ex.setIndices(inliers);
    ex.setNegative(true);
    ex.filter(*cloud_cropped);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr cloud_obst {new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr cloud_plane {new pcl::PointCloud<PointT>()};

    for (int index : inliers->indices)
    {
        cloud_plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_plane);
    
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment largest planar component from input cloud
    seg.setInputCloud(cloud);
    seg.segment( *inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate planar model for given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setClusterTolerance(clusterTolerance);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator p_it = it->indices.begin(); p_it != it->indices.end(); ++p_it)
                {
                    cluster->points.push_back(cloud->points[*p_it]);
                }
            clusters.push_back(cluster);
        }

    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    float max_z = maxPoint.z;
    float min_z = minPoint.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPlaneProjection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients());

    coeffs->values.resize(4);
    coeffs->values[0] = coeffs->values[1] = 0;
    coeffs->values[2] = 1.0;
    coeffs->values[3] = 0;
    
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cluster);
    proj.setModelCoefficients(coeffs);
    proj.filter(*clusterPlaneProjection);

    // Compute centroid

    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clusterPlaneProjection, pcaCentroid);
    Eigen::Matrix3f cov;
    computeCovarianceMatrixNormalized(*clusterPlaneProjection, pcaCentroid, cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVecPCA = eigen_solver.eigenvectors();
    

    eigenVecPCA.col(2) = eigenVecPCA.col(0).cross(eigenVecPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    
    projectionTransform.block<3,3>(0,0) = eigenVecPCA.transpose();
    projectionTransform.block<3, 1>(0,3) =  -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected ( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*clusterPlaneProjection, *cloudPointsProjected, projectionTransform);
    
    pcl::PointXYZ minPointProj, maxPointProj;
    pcl::getMinMax3D(*cloudPointsProjected, minPointProj, maxPointProj);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPointProj.getVector3fMap() + minPointProj.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVecPCA);
    const Eigen::Vector3f bboxTransform = eigenVecPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ box;
    box.cube_length = min_z + max_z;//maxPointProj.x - minPointProj.x;
    box.cube_width = maxPointProj.y - minPointProj.y;
    box.cube_height = maxPointProj.z - minPointProj.z;
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
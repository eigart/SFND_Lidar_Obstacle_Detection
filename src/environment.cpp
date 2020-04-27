/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // // ----------------------------------------------------
    // // -----Open 3D viewer and display simple highway -----
    // // ----------------------------------------------------
    
    // // RENDER OPTIONS
    // bool renderScene = false;
    // std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // // TODO:: Create lidar sensor

    // Lidar* theLidar (new Lidar(cars, 0));

    // // TODO:: Create point processor
    // pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = theLidar->scan();
    
    // ProcessPointClouds<pcl::PointXYZ>* pclProc (new ProcessPointClouds<pcl::PointXYZ>());

    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pclProc->SegmentPlane(inputCloud, 100, 0.2);
    // // renderPointCloud(viewer, segResult.first, "obstacles", Color(1,0,0));
    // // renderPointCloud(viewer, segResult.second, "plane", Color(0,1,0));

    // std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pclProc->Clustering(segResult.first, 1.0, 3, 30);

    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "cluster size:";
    //     pclProc->numPoints(cluster);
    //     renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % (int)colors.size()]);
    //     BoxQ box = pclProc->BoundingBoxQ(cluster);
    //     renderBox(viewer, box, clusterId, Color(1,0,0), 1.0);
    //     ++clusterId;
    // }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pclProc, const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    
    //ProcessPointClouds<pcl::PointXYZI>* pclProc (new ProcessPointClouds<pcl::PointXYZI>());
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pclProc->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pclProc->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-20.f, -6.f, -2.f,1.0), Eigen::Vector4f(20.f,8.f,2.f,1.0));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pclProc->SegmentPlane(filterCloud, 100, 0.2);
    
    
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pclProc->Clustering(segResult.first, 0.5, 10, 300);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size:";
        pclProc->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % (int)colors.size()]);
        Box box = pclProc->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0), 1.0);
        ++clusterId;
    }

    renderPointCloud(viewer, segResult.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segResult.second, "planeCloud", Color(0,1,0));
}


void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box b, uint clusterID)
{
    viewer->addCube(b.x_min, b.x_max, b.y_min, b.y_max, b.z_min, b.z_max, .2, .2, .2, "bBox");
}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcesorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcesorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIt = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {   
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcesorI->loadPcd((*streamIt).string());
        std::cout << inputCloudI << std::endl;
        cityBlock(viewer, pointProcesorI, inputCloudI);

        streamIt++;
        if (streamIt == stream.end())
        {
            streamIt = stream.begin();
        }

        viewer->spinOnce ();
    } 
}
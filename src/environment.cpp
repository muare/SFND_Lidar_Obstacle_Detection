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
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "cloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = processor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segResult.first, "plane", Color(0,1,0));
    //renderPointCloud(viewer, segResult.second, "obs", Color(1,0,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor->Clustering(segResult.second, 1., 3, 30 );
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* processor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, cloud, "cityblock");
    processor->numPoints(cloud);

    // Box box = processor->BoundingBox(cloud);
    // Box box;
    // box.x_min=-20;
    // box.y_min=-8;
    // box.z_min=-10;
    // box.x_max=30;
    // box.y_max=8;
    // box.z_max=3;
    // renderBox(viewer, box, 0);

    // std::cout << box.x_min <<","<<box.y_min<<","<<box.z_min <<endl;
    // std::cout << box.x_max <<","<<box.y_max<<","<<box.z_max <<endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processor->FilterCloud(cloud,0.15,Eigen::Vector4f(-10.,-6.,-3.5,1.),Eigen::Vector4f(30.,6.,3.,1.));
    renderPointCloud(viewer, filteredCloud, "filter cloud");
    processor->numPoints(filteredCloud);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = processor->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segResult.first, "plane", Color(0,1,0));
    // renderPointCloud(viewer, segResult.second, "obs", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processor->Clustering(segResult.second, .3, 50, 10000 );
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
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
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
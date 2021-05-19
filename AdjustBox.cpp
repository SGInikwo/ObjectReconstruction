//
// Created by MacBook2015 on 03/03/2021.
//
#include "AdjustBox.h"

using namespace std;
using namespace cv;

AdjustBox::AdjustBox(string inputStructure, string outputFilteredStructure, string bboxFile, float neighborsPercent, double stdDevThr){
    this->neighborsPercent = neighborsPercent;
    this->stdDevThr = stdDevThr;
    this->inputStructure = inputStructure;
    this->outputFilteredStructure = outputFilteredStructure;
    this->bboxFile = bboxFile;

    adjustingBox();
}
void AdjustBox::adjustingBox (){


    // Initialize parameters


    //   inputStructure = sfm/structure_raw.ply;
    //   outputFilteredStructure = sfm/structure_filt.ply;
    //   bboxFile = sfm/bbox_filt.txt;

    std::cout << "\nYou called:\n\n"
              << "	--i     <input .ply file with raw structure>        " + inputStructure  + "\n"
              << "	--o     <output .ply file with filtered structure>  " + outputFilteredStructure + "\n"
              << "	--bbox  <output txt file with bbox coordinates>     " + bboxFile + "\n"
              << "	--knn   <amount of neighbor points (less than 1)>   " << neighborsPercent << "\n"
              << "	--t     <deviation threshold>                       " << stdDevThr << "\n"
              << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    pcl::PLYWriter writer;

    // Read input structure point cloud
    reader.read(inputStructure.c_str(), *cloud);

    // Filter using the StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    float K = (float)(cloud->points.size()) * neighborsPercent;
    sor.setMeanK( (int)K );
    sor.setStddevMulThresh( stdDevThr );
    sor.filter(*cloud_filtered);

    float minX = 1e10;
    float maxX = -1e10;
    float minY = 1e10;
    float maxY = -1e10;
    float minZ = 1e10;
    float maxZ = -1e10;

    // Compute the new bbox from the filtered structure
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i){

        float xcoord = cloud_filtered->points[i].x;
        float ycoord = cloud_filtered->points[i].y;
        float zcoord = cloud_filtered->points[i].z;

        if (minX > xcoord) minX = xcoord;
        if (maxX < xcoord) maxX = xcoord;
        if (minY > ycoord) minY = ycoord;
        if (maxY < ycoord) maxY = ycoord;
        if (minZ > zcoord) minZ = zcoord;
        if (maxZ < zcoord) maxZ = zcoord;
    }

    float xmin = minX;
    float ymin = minY;
    float zmin = minZ;
    float xmax = maxX;
    float ymax = maxY;
    float zmax = maxZ;

    // Increase the new bbox by a certain margin in each direction
    // This is done to avoid losing info in case the filtering was too aggressive
    float margin = 0.1;
    xmin = xmin - (xmax - xmin) * margin;
    xmax = xmax + (xmax - xmin) * margin;
    ymin = ymin - (ymax - ymin) * margin;
    ymax = ymax + (ymax - ymin) * margin;
    zmin = zmin - (zmax - zmin) * margin;
    zmax = zmax + (zmax - zmin) * margin;

    // Write filtered structure (useful to debug)
    writer.write( outputFilteredStructure.c_str(), *cloud_filtered );

    // Write output bbox
    std::ofstream myFile;
    myFile.open(bboxFile.c_str());
    myFile << xmin << " " << ymin << " " << zmin << "\n";
    myFile << xmax << " " << ymax << " " << zmax << "\n";
    myFile.close();

    std::cout << "Done with Adjusting the Box" << endl;
}
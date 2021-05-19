//
// Created by MacBook2015 on 03/03/2021.
//
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "camera.h"
#ifndef OBJRECON_BBOX_H
#define OBJRECON_BBOX_H
class BoundingBox {

private:

    void Update();                             // Function to update the coordinates, nx, ny, nz and nvoxels attributes

public:

    cv::String filename;                           // Name of the txt file containing the bbox information
    cv::Mat coordinates;                           // 3x8 matrix containing the coordinates of the 8 vertices delimiting the bbox
    float xmin, xmax, ymin, ymax, zmin, zmax;  // Maximum and minimum coordinates of the bbox in each dimension (X,Y,Z)
    float voxelSize;                           // Length of the voxel edges
    int nx, ny, nz;                            // Number of voxels in each direction (X,Y,Z)
    int nvoxels;                               // Total number of voxels

    BoundingBox(cv::String bboxFile, float resolution);  // Constructor -> Build bbox from txt file
    BoundingBox();                                   // Function to initialize a bbox
    void Save(cv::String fileToSave);                    // Function to save the bbox in a txt file
    void SavePLY(cv::String fileToSave);                 // Function to save the bbox in a ply file
    void AdjustFromDepth(std::vector<Camera> P, std::vector< cv::Mat_<float> > depthmap, int minN);  // Adjust bbox using depth reliability
};
#endif //OBJRECON_BBOX_H

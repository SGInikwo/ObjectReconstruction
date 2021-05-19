//
// Created by MacBook2015 on 03/03/2021.
//
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#ifndef OBJRECON_CAMERA_H
#define OBJRECON_CAMERA_H
class Camera {

private:

    void Update();              // Function used to update the camera center and orientation

public:

    cv::String filename;            // Name of the txt file containing the camera information
    cv::Mat P;                      // Projection matrix
    cv::Mat K;                      // Intrinsics matrix
    cv::Mat R, t;                   // Rotation matrix (R) and translation vector (t)
    cv::Mat center, orientation;    // Optical center and orientation vector
    double mindepth, maxdepth;  // Minimum depth and maximum depth of the bbox in the camera reference system
    int h, w;                   // Height and width of the camera
    int index;                  // Index of the current camera in the set of N views
    int format;                 // Format of the input camera file (1 or 2)

    Camera(cv::String cameraFile, int viewInd);              // Constructor -> Build camera from txt file
    void Resize(float scale);                            // Function to resize camera
    void GetMinMaxDepth(cv::Mat bboxWorldCoord);             // Function to get mindepth and maxdepth attributes
    void DrawCamera(std::string outputPLY, float scale); // Function to draw a camera in a ply file
    void GetClosestCamera(std::vector<Camera> P, cv::Mat_<int> depthmapList, int* closestCameraInd); //Function to find the closest camera


};
#endif //OBJRECON_CAMERA_H

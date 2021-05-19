//
// Created by MacBook2015 on 03/03/2021.
//
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "camera.h"
#include "bbox.h"
#ifndef OBJRECON_POINTCLOUD_H
#define OBJRECON_POINTCLOUD_H
float getMedian(std::vector<float> f);
float getMean(std::vector<float> f);
void computePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat_<float> f, float margin, BoundingBox bbox);
void paintPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     std::vector<int> views_ind, std::vector<cv::Mat> oResizedImages, std::vector<Camera> P, std::string paintingMethod);
void writeBasicPointCloudPLY( std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writeRGBPointCloudPLY( std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void saveDistancefieldPLY(cv::Mat_<float> distancefield, float margin, std::string outputPLY, BoundingBox bbox,
                          bool COMPUTE_COLOR, std::vector<int> views_ind, std::vector<cv::Mat> oResizedImages, std::vector<Camera> P);
void writeIsoSurfaceRAW( cv::String filename, cv::Mat f );
void writeIsoSurfaceVTK( cv::String filename, cv::Mat f, int nx, int ny, int nz );
#endif //OBJRECON_POINTCLOUD_H

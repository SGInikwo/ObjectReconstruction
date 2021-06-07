//
// Created by MacBook2015 on 03/03/2021.
//
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <getopt.h>
#include <string>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

#ifndef OBJRECON_ADJUSTBOX_H
#define OBJRECON_ADJUSTBOX_H


class AdjustBox {
public:
    AdjustBox(std::string inputStructure, std::string outputFilteredStructure, std::string bboxFile, float neighborsPercent, double stdDevThr);
    ~AdjustBox();

    void adjustingBox ();

    float neighborsPercent;
    double stdDevThr = 1.0;
    std::string inputStructure, outputFilteredStructure, bboxFile;
};


#endif //OBJRECON_ADJUSTBOX_H

//
// Created by MacBook2015 on 03/03/2021.
//
#pragma once
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <math.h>
#include <getopt.h>
#include "camera.h"
#include "bbox.h"
#include "planesweep.h"
#include "tvl1.h"
#include "pointcloud.h"


#ifndef OBJRECON_REC3D_H
#define OBJRECON_REC3D_H


class Rec3D {
public:
    Rec3D(cv::String imagesDir, cv::String sfmDir, cv::String outputDir, bool COMPUTE_DEPTH, bool USE_MASK, bool COMPUTE_TSDF, bool COMPUTE_TVL1, bool COMPUTE_TVL2, bool DEBUG);
    ~Rec3D();

    int rec3D();

    bool DEBUG;
    bool COMPUTE_DEPTH;
    bool USE_MASK;
    bool COMPUTE_TSDF;
    bool COMPUTE_TVL1;
    bool COMPUTE_TVL2;
    bool TVL2_TVL1 = false;

    float scale = 1.0;
    cv::String costMeasure = "SSD"; // 'costMeasure' is the similarity measure used to compute the depthmaps (NCC, SSD or SAD)
    float conf = 0.3;           // 'conf' is the minimum cost that is necessary to consider a depth measure reliable
    int windowSize = 13;        // 'windowSize' is the length of one side of the window used in the similarity measure
    int nplanes = 150;
    float voxels = 1e7;         // 'voxelSize' is the length of the voxel edges
    double delta = 0.01;        // 'delta' is the uncertainity margin accepted in the distance fields
    double eta = 0.2;
    float lambda = 0.1;         // 'lambda' is the weight assigned to the data fidelity term
    float theta = 0.25;         // 'theta' is the weight assigned to the dual problem similarity constraint (u~=v)
    float tau = 0.15;           // 'tau' is the step used for the optimization procedure. It should not be larger than 0.16
    int maxIt = 50;             // 'maxIt' is the maximum number of iterations allowed in the optimization procedure
    float tolerance = 0.0001;   // 'tolerance' is a convergence threshold used in the optimization procedure

    cv::String imagesDir;
    cv::String sfmDir;
    cv::String outputDir;
    cv::String depthDir;
    cv::String tsdfDir;
    cv::String tvl1Dir;
    cv::String tvl2Dir;

    int64 time1, time2;
    double time;
};


#endif //OBJRECON_REC3D_H

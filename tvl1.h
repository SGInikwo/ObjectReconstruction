//
// Created by MacBook2015 on 03/03/2021.
//
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <omp.h>
#include "camera.h"
#include "bbox.h"

#ifndef OBJRECON_TVL1_H
#define OBJRECON_TVL1_H
void divergence(cv::Mat_<float> divp, cv::Mat_<float> p1, cv::Mat_<float> p2, cv::Mat_<float> p3, int nx, int ny, int nz);
void forwardGradient(cv::Mat f, cv::Mat fx, cv::Mat fy, cv::Mat fz, int nx, int ny, int nz);
void solveTVL2(cv::Mat_<float> u, cv::Mat* tvl2, BoundingBox bbox, std::vector< cv::Mat_<float> > distancefield,
               float lambda, float theta, int maxIterations, float tau, float tolerance,
               cv::String tvl2Dir, std::vector<cv::Mat> Icolor, std::vector<Camera> P, bool SAVE_CLOUD);
void solveTVL1(cv::Mat_<float> u, cv::Mat* tvl1, BoundingBox bbox, std::vector< cv::Mat_<float> > distancefield,
               float lambda, float theta, int maxIterations, float tau, float tolerance,
               cv::String tvl1Dir, std::vector<cv::Mat> Icolor, std::vector<Camera> P, bool SAVE_CLOUD);

#endif //OBJRECON_TVL1_H

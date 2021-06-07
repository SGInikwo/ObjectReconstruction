//
// Created by MacBook2015 on 03/03/2021.
//
#include <opencv2/opencv.hpp>
#include <unistd.h>
#ifndef OBJRECON_PLANESWEEP_H
#define OBJRECON_PLANESWEEP_H
std::vector<float> linspace(float a, float b, int N);
cv::Mat_<float> computeNCC(cv::Mat image1, cv::Mat image2, int windowSize);
cv::Mat_<float> computeSSD(cv::Mat im1, cv::Mat im2, int windowSize);
cv::Mat_<float> computeSAD(cv::Mat im1, cv::Mat im2, int windowSize);
void planeSweep(cv::Mat I1, cv::Mat I2, cv::Mat P1, cv::Mat P2, cv::Mat M, cv::Mat* outputDepthmap, cv::String similarityMeasure,
                float conf, float mindepth, float maxdepth, int nplanes, int windowSize);
void saveDephtmapPNG(cv::Mat_<float> depthmap, std::string outputPNGfilename );
void addSaltPepperNoiseDephtmap(cv::Mat inputCleanDepthmap, cv::Mat* outputNoisyDepthmap);
#endif //OBJRECON_PLANESWEEP_H

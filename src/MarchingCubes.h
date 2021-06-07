//
// Created by MacBook2015 on 03/03/2021.
//

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <getopt.h>

#include "vtkSmartPointer.h"
#include "vtkStructuredPointsReader.h"
#include "vtkMarchingCubes.h"
#include "vtkPolyDataConnectivityFilter.h"
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkTriangle.h>

#ifndef OBJRECON_MARCHINGCUBES_H
#define OBJRECON_MARCHINGCUBES_H

using namespace std;
using namespace cv;

class MarchingCubes {
public:
    MarchingCubes(string inputIsoSurface, string outputMesh, bool toObj);
    ~MarchingCubes();

    void marchCubPLY();
    void marchCubOBJ();

    string inputIsoSurface, outputMesh;

    float isoValue = 50;
    bool LARGEST_REGION = false;
};


#endif //OBJRECON_MARCHINGCUBES_H

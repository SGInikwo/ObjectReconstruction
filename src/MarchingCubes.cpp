//
// Created by MacBook2015 on 03/03/2021.
//

#include "MarchingCubes.h"

MarchingCubes::MarchingCubes(string inputIsoSurface, string outputMesh, bool toObj){
    this->inputIsoSurface = inputIsoSurface;
    this->outputMesh = outputMesh;

    if(!toObj)
        marchCubPLY();
    else
        marchCubOBJ();
}

void MarchingCubes::marchCubPLY(){

    int64 e1 = getTickCount();
    cout << "\nRunning marching cubes...\n" << endl;

    // Load data
    vtkSmartPointer<vtkStructuredPointsReader> reader = vtkSmartPointer<vtkStructuredPointsReader>::New();
    reader->SetFileName(inputIsoSurface.c_str());

    // Run marching cubes
    vtkSmartPointer<vtkMarchingCubes> mc = vtkSmartPointer<vtkMarchingCubes>::New();
    mc->SetInputConnection(reader->GetOutputPort());
    mc->ComputeNormalsOn();
    mc->ComputeGradientsOn();
    mc->SetValue(0, isoValue);  // isoValue acts as threshold
    mc->Update();

    // Extract the mesh from the marching cubes output
    // If the user has enabled the 'LARGEST_REGION' option, then only the largest continuous part
    // of the 3D model will be extracted with vtkPolyDataConnectivityFilter, while the rest will be discarded
    vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataConnectivityFilter> confilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    // The triangleFilter below will be used later to get the indeces of the 3 vertices forming each face
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();

    if (LARGEST_REGION){
        // To pick only largest continuous region
        confilter->SetInputConnection(mc->GetOutputPort());
        confilter->SetExtractionModeToLargestRegion();
        confilter->Update();
        mesh = confilter->GetOutput();
        triangleFilter->SetInputConnection(confilter->GetOutputPort());

    }
    else{
        mesh = mc->GetOutput();
        triangleFilter->SetInputConnection(mc->GetOutputPort());
    }
    triangleFilter->Update();

    // Get number of vertices and faces
    int numberOfVertices = mesh->GetNumberOfPoints();
    int numberOfFaces = mesh->GetNumberOfCells();
    cout << "  Output mesh contains " << numberOfVertices << " vertices and " << numberOfFaces << " triangular faces" << endl;

    // Write output ply file with the extracted mesh
    ofstream outFile( outputMesh.c_str());
    // Write file header
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " << numberOfVertices << endl;
    outFile << "property float x" << endl;
    outFile << "property float y" << endl;
    outFile << "property float z" << endl;
    outFile << "element face " << numberOfFaces << endl;
    outFile << "property list uchar int vertex_indices" << endl;
    outFile << "end_header" << endl;
    // Write vertices
    for(int i = 0; i < numberOfVertices; i++){
        double p[3];
        mesh->GetPoint(i,p);
        outFile << p[0] << " " << p[1] << " " << p[2] << " " << endl;
    }
    // Write faces
    for (vtkIdType i = 0; i < numberOfFaces; i++){
        // Get ids of the face vertices
        vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
        triangleFilter->GetOutput()->GetCellPoints(i, cellPointIds);
        int v0_ind = cellPointIds->GetId(0);
        int v1_ind = cellPointIds->GetId(1);
        int v2_ind = cellPointIds->GetId(2);
        outFile << "3 " << v0_ind << " " << v2_ind << " " << v1_ind << endl;
    }
    outFile.close();

    int64 e2 = getTickCount();
    double time = (e2 - e1)/getTickFrequency();
    cout << "\nMarching cubes successfully completed in " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;
}

void MarchingCubes::marchCubOBJ() {

    int64 e1 = getTickCount();
    cout << "\nRunning marching cubes...\n" << endl;

    // Load data
    vtkSmartPointer<vtkStructuredPointsReader> reader = vtkSmartPointer<vtkStructuredPointsReader>::New();
    reader->SetFileName(inputIsoSurface.c_str());

    // Run marching cubes
    vtkSmartPointer<vtkMarchingCubes> mc = vtkSmartPointer<vtkMarchingCubes>::New();
    mc->SetInputConnection(reader->GetOutputPort());
    mc->ComputeNormalsOn();
    mc->ComputeGradientsOn();
    mc->SetValue(0, isoValue);  // isoValue acts as threshold
    mc->Update();

    // Extract the mesh from the marching cubes output
    // If the user has enabled the 'LARGEST_REGION' option, then only the largest continuous part
    // of the 3D model will be extracted with vtkPolyDataConnectivityFilter, while the rest will be discarded
    vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataConnectivityFilter> confilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    // The triangleFilter below will be used later to get the indeces of the 3 vertices forming each face
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();

    if (LARGEST_REGION){
        // To pick only largest continuous region
        confilter->SetInputConnection(mc->GetOutputPort());
        confilter->SetExtractionModeToLargestRegion();
        confilter->Update();
        mesh = confilter->GetOutput();
        triangleFilter->SetInputConnection(confilter->GetOutputPort());

    }
    else{
        mesh = mc->GetOutput();
        triangleFilter->SetInputConnection(mc->GetOutputPort());
    }
    triangleFilter->Update();

    // Get number of vertices and faces
    int numberOfVertices = mesh->GetNumberOfPoints();
    int numberOfFaces = mesh->GetNumberOfCells();
    cout << "  Output mesh contains " << numberOfVertices << " vertices and " << numberOfFaces << " triangular faces" << endl;

    // Write output ply file with the extracted mesh
    ofstream outFile( outputMesh.c_str());
    // Write file header
    outFile << "# File type: ASCII OBJ" << endl;

    // Write vertices
    for(int i = 0; i < numberOfVertices; i++){
        double p[3];
        mesh->GetPoint(i,p);
        outFile << "v "<< p[0] << " " << p[1] << " " << p[2] << " " << endl;
    }
    // Write faces
    for (vtkIdType i = 0; i < numberOfFaces; i++){
        // Get ids of the face vertices
        vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
        triangleFilter->GetOutput()->GetCellPoints(i, cellPointIds);
        int v0_ind = cellPointIds->GetId(0);
        int v1_ind = cellPointIds->GetId(1);
        int v2_ind = cellPointIds->GetId(2);
        outFile << "f " << (v0_ind + 1) << " " << (v2_ind + 1) << " " << (v1_ind + 1)<< endl;
    }
    outFile.close();

    int64 e2 = getTickCount();
    double time = (e2 - e1)/getTickFrequency();
    cout << "\nMarching cubes successfully completed in " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;
}
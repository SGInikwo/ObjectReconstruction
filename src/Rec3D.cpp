//
// Created by MacBook2015 on 03/03/2021.
//
#include "Rec3D.h"

using namespace cv;
using namespace std;

Rec3D::Rec3D(String imagesDir, String sfmDir, String outputDir, bool COMPUTE_DEPTH, bool USE_MASK, bool COMPUTE_TSDF, bool COMPUTE_TVL1, bool COMPUTE_TVL2, bool DEBUG) {
    // Initialize parameters

    this->DEBUG = DEBUG;        // 'DEBUG' mode displays all information regarding the loading of the input data and the resize step

    // PLANE SWEEP
    this->COMPUTE_DEPTH = COMPUTE_DEPTH; // 'COMPUTE_DEPTH' indicates if depthmaps will be computed using the plane sweep algorithm
    this->USE_MASK = USE_MASK;      // 'USE_MASK' indicates if the depthmaps will be filtered using masks

    // DISTANCE FIELDS
    this->COMPUTE_TSDF = COMPUTE_TSDF;  // 'COMPUTE_TSDF' indicates if distance fields will be computed

    // TVL1-TVL2
    this->COMPUTE_TVL1 = COMPUTE_TVL1;  // 'COMPUTE_TVL1' indicates if depth fusion with TVL1 will be computed
    this->COMPUTE_TVL2 = COMPUTE_TVL2;  // 'COMPUTE_TVL2' indicates if depth fusion with TVL2 will be computed

    // Directories to be used
    this->imagesDir = imagesDir;
    this->sfmDir = sfmDir;
    this->outputDir = outputDir;
    depthDir = outputDir + "/depth";
    tsdfDir = outputDir + "/tsdf";
    tvl1Dir = outputDir + "/tvl1";
    tvl2Dir = outputDir + "/tvl2";

    rec3D();
}

int Rec3D::rec3D(){
    cout << " _______________________________________________________________________________________\n"
         << "|________________________________________rec3D__________________________________________\n"
         << "|                                                                                       \n"
         << "| You called:                                                                           \n"
         << "|                                                                                       \n"
         << "|	     --img     <input images directory>           " + imagesDir + "\n"
         << "|	     --sfm     <input sfm directory>              " + sfmDir + "\n"
         << "|	     --out     <output directory>                 " + outputDir + "\n"
         << "|	     --s       <scale factor>                     " << scale << "\n";
    cout << "|	     --depth   <compute depth>                    "; (!COMPUTE_DEPTH) ? cout << "OFF\n" : cout << "ON\n";
    cout << "|	     --tsdf    <compute distance fields>          "; (!COMPUTE_TSDF)  ? cout << "OFF\n" : cout << "ON\n";
    cout << "|	     --tvl2    <compute depth fusion with TVL2>   "; (!COMPUTE_TVL2)  ? cout << "OFF\n" : cout << "ON\n";
    cout << "|	     --tvl1    <compute depth fusion with TVL1>   "; (!COMPUTE_TVL1)  ? cout << "OFF\n" : cout << "ON\n";
    cout << "|	     --fast    <use TVL2 to initialize TVL1>      "; (!TVL2_TVL1)     ? cout << "OFF\n" : cout << "ON\n";
    cout << "|	     --debug   <debug mode>                       "; (!DEBUG)         ? cout << "OFF\n" : cout << "ON\n";
    cout << "|_______________________________________________________________________________________\n" << endl;

    // *****************************************************
    // ***************** CHECK DIRECTORIES *****************
    // *****************************************************

    // Check if the input images directory exists
    if (system(("test -d "+imagesDir).c_str())!=0){
        cerr << "\nERROR: Input images directory does not exist." << endl;
        return EXIT_FAILURE;
    }
    // Check if the input sfm directory exists
    if (system(("test -d "+sfmDir).c_str())!=0){
        cerr << "\nERROR: Input sfm directory does not exist." << endl;
        return EXIT_FAILURE;
    }
    // Check if the output directory exists. If it does not exist, then create it.
    if (system(("test -d "+outputDir).c_str())!=0){
        system(("mkdir "+outputDir).c_str());
    }
    // Check if the output depthmaps directory exists. If it does not exist, then create it.
    // If it does exist and new depthmaps are to be computed, then remove it and create a new one.
    if (system(("test -d "+depthDir).c_str())!=0){
        system(("mkdir "+depthDir).c_str());
    }
    else{
        if(COMPUTE_DEPTH){
            system(("rm -r "+depthDir).c_str());
            system(("mkdir "+depthDir).c_str());
        }
    }
    // Check if the output distance fields directory exists. If it does not exist, then create it.
    // If it does exist and new distance fields are to be computed, then remove it and create a new one.
    if (system(("test -d "+tsdfDir).c_str())!=0){
        system(("mkdir "+tsdfDir).c_str());
    }
    else{
        if(COMPUTE_TSDF){
            system(("rm -r "+tsdfDir).c_str());
            system(("mkdir "+tsdfDir).c_str());
        }
    }
    // Check if the output tvl1 directory exists. If it does not exist, then create it.
    // If it does exist and a new tvl1 fusion is to be computed, then remove it and create a new one.
    if (system(("test -d "+tvl1Dir).c_str())!=0){
        system(("mkdir "+tvl1Dir).c_str());
    }
    else{
        if(COMPUTE_TVL1){
            system(("rm -r "+tvl1Dir).c_str());
            system(("mkdir "+tvl1Dir).c_str());
        }
    }
    // Check if the output tvl2 directory exists. If it does not exist, then create it.
    // If it does exist and a new tvl2 fusion is to be computed, then remove it and create a new one.
    if (system(("test -d "+tvl2Dir).c_str())!=0){
        system(("mkdir "+tvl2Dir).c_str());
    }
    else{
        if(COMPUTE_TVL2){
            system(("rm -r "+tvl2Dir).c_str());
            system(("mkdir "+tvl2Dir).c_str());
        }
    }


    // *****************************************************
    // *************** LOAD INPUT IMAGES *******************
    // *****************************************************

    streambuf* orig_buf = cout.rdbuf();
    if(!DEBUG) cout.rdbuf(NULL);

    cout << "\nLoading input data..." << endl;
    time1 = getTickCount();

    // Get input images filenames (and masks if provided)
    vector<String> pngFiles;   // Vector to store the name of all the input png images (including masks if provided)
    vector<String> imageFiles; // Vector to store the name of the input images that are a view of the object to reconstruct
    vector<String> maskFiles;  // Vector to store the name of the input images that are masks (if provided)

    glob(imagesDir+"/*.jpg", pngFiles); // Get all the png files in the input images directory
    if (pngFiles.size() == 0){
        cerr << "\nERROR: No jpg images were found." << endl;
        return EXIT_FAILURE;
    }
    else{
        // There were png images found, now fill the imageFiles and maskFiles vectors.
        // Images ending with "_mask.png" are masks, the rest are views of the object to reconstruct.
        for(int i = 0; i < pngFiles.size(); i++){
            if (pngFiles[i].find("_mask.jpg") < pngFiles[i].size()){
                maskFiles.push_back(pngFiles[i]);
            }
            else{
                imageFiles.push_back(pngFiles[i]);
            }
        }
    }

    // Check that input views were found (it is possible to have no masks, but with no views the program would crash)
    if (imageFiles.size() == 0){
        cerr << "\nERROR: No input images were found." << endl;
        return EXIT_FAILURE;
    }
    // If USE_MASK is set to true, check that the number of input masks is the same as the number of input views
    if (USE_MASK && !maskFiles.empty() && imageFiles.size() != maskFiles.size()){
        cerr << "\nERROR: The number of input views is different from the number of input masks." << endl;
        return EXIT_FAILURE;
    }

    // Load input images (and masks if provided)
    vector<Mat> oImages;  // Vector to store the original input images (with their original size and original color)
    vector<Mat> oMasks;   // Vector to store the original black and white masks (if provided)

    // Fill vector oImages
    cout << "\nLoading images...\n" << endl;
    for (int i = 0; i < imageFiles.size(); i++){
        // Load current image
        oImages.push_back( imread(imageFiles[i], IMREAD_COLOR) );
        cout << "  " << imageFiles[i] << endl;
        // Check that no error occurred
        if(!oImages.back().data){
            cerr << "\nERROR: There was a problem loading the input images." << endl;
            return EXIT_FAILURE;
        }
    }

    // Get original images size
    int h = oImages[0].rows;
    int w = oImages[0].cols;

    // Fill vector oMasks
    if (USE_MASK) cout << "\nLoading masks...\n" << endl;
    if (USE_MASK && !maskFiles.empty()){
        // If USE_MASK is set to true and input masks are indeed provided, then load input masks
        for (int i = 0; i < maskFiles.size(); i++){
            // Load current mask
            oMasks.push_back( imread(maskFiles[i], IMREAD_GRAYSCALE) );
            cout << "  " << maskFiles[i] << endl;
            // Check that no error occurred
            if(!oMasks.back().data){
                cerr << "\nERROR: There was a problem loading the input masks." << endl;
                return EXIT_FAILURE;
            }
        }
    }
    else{
        // If no input masks are provided, then load default masks covering all input images size
        if (USE_MASK) cout << "  WARNING: No input masks were found. Using default masks." << endl;
        for (int i = 0; i < imageFiles.size(); i++){
            oMasks.push_back( Mat::ones(h, w, CV_32FC1) );
        }
    }

    // Save txt file listing the input image files paths (this will help painting the output mesh later)
    String imagesList = outputDir + "/images.txt";
    system(("if [ -f "+imagesList+" ]; then rm "+imagesList+"; fi").c_str()); // If file already exists, then remove it
    // Write file
    ofstream imagesfile;
    imagesfile.open(imagesList.c_str());
    for (int i = 0; i < imageFiles.size(); i++)
        imagesfile << "../" << imageFiles[i] << "\n";
    imagesfile.close();


    // *****************************************************
    // *************** LOAD INPUT CAMERAS ******************
    // *****************************************************

    // Get input camera filenames
    vector<String> cameraFiles;  // Vector to store the name of the input camera files
    glob(sfmDir+"/*_cam.txt", cameraFiles); // Fill cameraFiles

    // Check that the number of input cameras is the same as the number of input images
    if( cameraFiles.size() != imageFiles.size() ){
        cerr << "\nERROR: The number of camera files and image files is not the same." << endl;
        return EXIT_FAILURE;
    }

    // Load input cameras
    cout << "\nLoading cameras...\n" << endl;
    vector<Camera> oP;  // Vector to store the original input cameras
    for(int i = 0; i < cameraFiles.size(); i++){
        // Load current camera
        oP.push_back( Camera(cameraFiles[i],i) );
        cout << "  " << cameraFiles[i] << endl;
    }

    // Save txt file listing the input camera files paths (this will help painting the output mesh later)
    String camerasList = outputDir + "/cameras.txt";
    system(("if [ -f "+camerasList+" ]; then rm "+camerasList+"; fi").c_str()); // If file already exists, then remove it
    // Write file
    ofstream camerasfile;
    camerasfile.open (camerasList.c_str());
    for (int i = 0; i < cameraFiles.size(); i++)
        camerasfile << "../" << cameraFiles[i] << "\n";
    camerasfile.close();

    // Save cameras as ply files (useful to debug)
    if (system(("test -d "+outputDir+"/cameras").c_str())==0){
        system(("rm -r "+outputDir+"/cameras").c_str());
    }
    system(("mkdir "+outputDir+"/cameras").c_str());
    for (int v = 0; v < cameraFiles.size(); v++){
        ostringstream cameraPNG;
        cameraPNG << "cp "+imageFiles[v]+" "+outputDir+"/cameras/cam_" << v << ".jpg";
        system(cameraPNG.str().c_str());
        ostringstream outputCameraPLY;
        outputCameraPLY << outputDir << "/cameras/cam_" << v << ".ply";
        oP[v].DrawCamera( outputCameraPLY.str(), 0.2 );
    }

    // *****************************************************
    // *************** LOAD INPUT BBOX *********************
    // *****************************************************

    cout << "\nLoading bbox...\n" << endl;

    BoundingBox bbox;

    // Read bbox from bbox_raw.txt file in the 'sfm' folder and save it as ply (useful to debug)
    bbox = BoundingBox(sfmDir + "/bbox_raw.txt", voxels);
    bbox.SavePLY(outputDir + "/bbox_raw.ply");

    cout << "  Raw bbox: " << endl;
    cout << "    - Voxel resolution: " << bbox.voxelSize << endl;
    cout << "    - Grid dimensions: " << bbox.nx << "x" << bbox.ny << "x" << bbox.nz << " voxels" << endl;
    cout << "    - Number of voxels: " << bbox.nvoxels << endl;

    // Read bbox from bbox_filt.txt file in the 'sfm' folder and save it as ply (useful to debug)
    bbox = BoundingBox(sfmDir + "/bbox_filt.txt", voxels);
    bbox.SavePLY(outputDir + "/bbox_filt.ply");

    cout << "  Filtered bbox: " << endl;
    cout << "    - Voxel resolution: " << bbox.voxelSize << endl;
    cout << "    - Grid dimensions: " << bbox.nx << "x" << bbox.ny << "x" << bbox.nz << " voxels" << endl;
    cout << "    - Number of voxels: " << bbox.nvoxels << endl;

    // Save grid dimensions (this will help later writing the iso-surface vtk file used to extract the mesh)
    String grid = outputDir + "/grid.txt";
    system(("if [ -f "+grid+" ]; then rm "+grid+"; fi").c_str());
    ofstream gridfile;
    gridfile.open (grid.c_str());
    gridfile << bbox.nx << "\n";
    gridfile << bbox.ny << "\n";
    gridfile << bbox.nz << "\n";
    gridfile << bbox.xmin << "\n";
    gridfile << bbox.ymin << "\n";
    gridfile << bbox.zmin << "\n";
    gridfile << bbox.voxelSize << "\n";
    gridfile.close();


    // *****************************************************
    // ************** RISIZE INPUT IMAGES ******************
    // *****************************************************

    cout << "\nResizing input images..." << endl;

    int N = imageFiles.size(); // Get number of input views
    vector<Mat> I(N);          // Vector containing the resized grayscale version of the original images
    vector<Mat> M(N);          // Vector containing the resized version of the original masks
    vector<Mat> Icolor(N);     // Vector containing the resized color version of the original images
    vector<Camera> P = oP;     // Vector containing the resized cameras

    // Iterate for all images
    for(int i = 0; i < N; i++){
        resize(oImages[i], Icolor[i], I[i].size(), scale, scale); // Resize current image
        cvtColor(Icolor[i], I[i], COLOR_BGRA2GRAY);                   // Convert current image to grayscale
        resize(oMasks[i], M[i], M[i].size(), scale, scale);       // Resize associated mask
        P[i].Resize(scale);                                       // Resize associated camera matrix

        // Check that the size of all images is the same
        if (i>0 && (I[i].rows != I[i-1].rows || I[i].cols != I[i-1].cols) ){
            cerr << "\nERROR: The size of the input images is not the same." << endl;
            return EXIT_FAILURE;
        }
    }

    // Update images size
    h = I[0].rows;
    w = I[0].cols;

    cout << "\n  Scale factor: " << scale << endl;
    cout << "  Original (w,h): (" << oImages[0].cols << "," << oImages[0].rows << ")" << endl;
    cout << "  New (w,h): (" << w << "," << h << ")" << endl;

    time2 = getTickCount();
    time = (time2 - time1)/getTickFrequency();
    cout << "\nLoading input data took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;

    if(!DEBUG) cout.rdbuf(orig_buf);

    if (!COMPUTE_DEPTH && !COMPUTE_TSDF && !COMPUTE_TVL1 && !COMPUTE_TVL2) return EXIT_SUCCESS;


    // *****************************************************
    // ***************** COMPUTE DEPTHMAPS *****************
    // *****************************************************

    time1 = getTickCount();

    if(COMPUTE_DEPTH){

        cout << "\nComputing depthmaps..." << endl;

        cout << "\nYou called:\n\n"
             << "	--mask    <use masks>                        "; (!USE_MASK) ? cout << "OFF\n" : cout << "ON\n";
        cout << "	--planes  <number of frontoparallel planes>  " << nplanes << "\n"
             << "	--wsize   <window size>                      " << windowSize << "\n"
             << "	--cost    <similarity measure: NCC|SSD|SAD>  " << costMeasure << "\n"
             << "	--conf    <minimum depth confidence>         " << conf << "\n";
        cout << " " << endl;
    }

    vector< Mat_<float> > depthmap(N);             // We will build as much depthmaps as N views

    Mat_<int> depthmapViewsList;                   // This list will match each depthmap (referenced by the row number)
    // With the cameras used to build it with the planesweep algorithm
    // First column is reference camera, second column is second camera

    Mat_<int> firstPair = (Mat_<int>(1,2) << 0,0); // This pair initializes the depthmapViewsList
    depthmapViewsList.push_back( firstPair );      // This is simply done to avoid crash at first iteration. It will be removed later.

    if(COMPUTE_DEPTH){

        for(int i = 0; i < N; i++){

            // Get minimum and maximum depth of the bbox coordinates with respect the current camera
            P[i].GetMinMaxDepth(bbox.coordinates);
            float minDepth = P[i].mindepth;
            float maxDepth = P[i].maxdepth;

            // Get the camera closest to the current camera (select the second view to build the depthmap)
            int closestCameraInd;
            P[i].GetClosestCamera(P,depthmapViewsList,&closestCameraInd);
            Mat_<int> newPair = (Mat_<int>(1,2) << i,closestCameraInd);
            depthmapViewsList.push_back( newPair );

            // Display current depthmap information
            cout << "  (" << i << "," << closestCameraInd <<") min depth: " << minDepth << " max depth: " << maxDepth << endl;

            // Compute current depthmap using the plane sweep algorithm
            planeSweep(I[i], I[closestCameraInd], P[i].P, P[closestCameraInd].P, M[i], &depthmap[i],
                       costMeasure, conf, minDepth, maxDepth, nplanes, windowSize);

            // Save current depthmap as xml
            ostringstream outputDepthmapXML;
            outputDepthmapXML  << depthDir << "/depthmap_" << i << ".xml";
            FileStorage file(outputDepthmapXML.str().c_str(), FileStorage::WRITE);
            file << "Data" << depthmap[i];
            file.release();

            // Save current depthmap as png (useful to debug)
            ostringstream outputDepthmapPNG;
            outputDepthmapPNG  << depthDir << "/depthmap_" << i << ".png";
            saveDephtmapPNG(depthmap[i], outputDepthmapPNG.str());

        }
        cout << "\nDepthmaps successfully computed and stored at " << depthDir << endl;

        // Remove first pair of the depthmapViewsList
        depthmapViewsList = depthmapViewsList(Range(1,N+1),Range::all());

        // Save depthmaps list as xml
        ostringstream outputListXML;
        outputListXML  << depthDir << "/list.xml";
        FileStorage file(outputListXML.str().c_str(), FileStorage::WRITE);
        file << "Data" << depthmapViewsList;
        file.release();
    }
    else{

        // Read depthmaps
        for(int i = 0; i < N; i++){
            ostringstream outputDepthmapXML;
            outputDepthmapXML  << depthDir << "/depthmap_" << i << ".xml";
            FileStorage file(outputDepthmapXML.str().c_str(), FileStorage::READ);
            file["Data"] >> depthmap[i];
        }

        // Read depthmaps list
        ostringstream outputListXML;
        outputListXML << depthDir << "/list.xml";
        FileStorage file(outputListXML.str().c_str(), FileStorage::READ);
        file["Data"] >> depthmapViewsList;
    }

    time2 = getTickCount();
    time = (time2 - time1)/getTickFrequency();
    if(COMPUTE_DEPTH) cout << "\nComputing depthmaps took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;

    if (!COMPUTE_TSDF && !COMPUTE_TVL1 && !COMPUTE_TVL2) return EXIT_SUCCESS;


    // *****************************************************
    // ************* COMPUTE DISTANCE FIELDS ***************
    // *****************************************************

    time1 = getTickCount();

    if(COMPUTE_TSDF){

        cout << "\nComputing distance fields..." << endl;

        cout << "\nYou called:\n\n"
             << "	--voxels  <number of voxels in the bbox>     " << voxels << "\n"
             << "	--delta   <delta>                            " << delta << "\n"
             << "	--eta     <eta>                              " << eta << endl;
    }

    vector< Mat_<float> > distancefield;

    if(COMPUTE_TSDF){

        // Initialize distance fields
        // Each distance field is initialized as an array of 1xN voxels with all values = -5.0
        // -5.0 is a dedicated value used to indentify voxels with weight = 0
        for (int i=0; i < N; i++){
            distancefield.push_back( (-5.0)*Mat::ones(1, bbox.nvoxels, CV_32FC1) );
        }

        // Conversion from 2D depthmaps to 3D distance fields starts here
        omp_set_num_threads(omp_get_max_threads());
#pragma omp parallel for
        for (int z=0; z < bbox.nz; z++){
            for (int y=0; y < bbox.ny; y++){
                for (int x=0; x < bbox.nx; x++){

                    // Current voxel index (from 3D array to 1D array)
                    int ind = x + bbox.nx * (y + bbox.ny * z);

                    // Current voxel coordinates in the 3D space
                    float xcoord = x*bbox.voxelSize + bbox.xmin;
                    float ycoord = y*bbox.voxelSize + bbox.ymin;
                    float zcoord = z*bbox.voxelSize + bbox.zmin;

                    // For each depthmap
                    for (int v=0; v < N; v++){

                        // Project the voxel from the 3D space to the depthmap
                        Mat projection = P[v].P*(Mat_<float>(4,1) << xcoord,ycoord,zcoord,1.0);
                        float xp = projection.at<float>(0,0);
                        float yp = projection.at<float>(1,0);
                        float zp = projection.at<float>(2,0);
                        xp = cvRound(xp/zp);
                        yp = cvRound(yp/zp);

                        // If the voxel falls within the depthmap and projects into a pixel with reliable depth, then compute the TSDF
                        // If the TSDF is bigger than -eta, then the voxel is assigned weigth 1
                        // Otherwise the voxel remains assigned value -5.0, which is the dedicated value meaning it has weight 0
                        if (xp >= 0 && xp < w && yp >= 0 && yp < h && depthmap[v].at<float>(yp,xp) != 255.0){
                            double TSDF = depthmap[v].at<float>(yp,xp) - zp;
                            if (TSDF > -eta) distancefield[v].at<float>(ind) = max(-1.0,min(1.0,TSDF/delta));
                        }
                    }

                }
            }
        }

        // Save distance fields
        for (int v=0; v < N; v++){

            // Save current distance field as xml
            ostringstream outputDistancefieldXML;
            outputDistancefieldXML << tsdfDir << "/tsdf_" << v << ".xml";
            FileStorage file(outputDistancefieldXML.str().c_str(), FileStorage::WRITE);
            file << "Data" << distancefield[v];
            file.release();

            if (DEBUG){
                // Save current distance field as ply (useful to debug)
                ostringstream outputDistancefieldPLY;
                outputDistancefieldPLY << tsdfDir << "/tsdf_" << v << ".ply";
                vector<int> views_ind;
                views_ind.push_back(depthmapViewsList.at<int>(v,0));
                views_ind.push_back(depthmapViewsList.at<int>(v,1));
                saveDistancefieldPLY(distancefield[v], 0.3, outputDistancefieldPLY.str(), bbox, true, views_ind, Icolor, P);
            }
        }
        cout << "\nDistance fields successfully computed and stored at " << tsdfDir << endl;

    }
    else{

        // Read distance fields
        for (int v=0; v < N; v++){
            Mat_<float> currentTSDF;
            ostringstream outputDistancefieldXML;
            outputDistancefieldXML << tsdfDir << "/tsdf_" << v << ".xml";
            FileStorage file(outputDistancefieldXML.str().c_str(), FileStorage::READ);
            file["Data"] >> currentTSDF;
            distancefield.push_back(currentTSDF);
        }
    }

    time2 = getTickCount();
    time = (time2 - time1)/getTickFrequency();
    if(COMPUTE_TSDF) cout << "\nComputing distance fields took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;

    if (!COMPUTE_TVL1 && !COMPUTE_TVL2) return EXIT_SUCCESS;


    // *****************************************************
    // **************** TVL2 OPTIMIZATION ******************
    // *****************************************************

    time1 = getTickCount();

    if(COMPUTE_TVL2){

        cout << "\nComputing depth fusion with TVL2..." << endl;

        cout << "\nYou called:\n\n"
             << "	--lambda  <lambda>                           " << lambda << "\n"
             << "	--theta   <theta>                            " << theta << "\n"
             << "	--tau     <tau>                              " << tau << "\n"
             << "	--maxit   <maximum iterations>               " << maxIt << endl;
    }

    Mat tvl2;

    if(COMPUTE_TVL2){
        // Initialize the output implicit surface resulting from the depth fusion
        Mat_<float> u = Mat::ones(1, bbox.nvoxels, CV_32FC1);
        // Compute depth fusion optimizing TVL2 energy functional
        if(TVL2_TVL1){
            solveTVL2(u, &tvl2, bbox, distancefield, lambda, theta, 5, tau, tolerance, tvl2Dir, Icolor, P, false);
        }
        else{
            solveTVL2(u, &tvl2, bbox, distancefield, lambda, theta, maxIt, tau, tolerance, tvl2Dir, Icolor, P, DEBUG);
        }
        cout << "\nDepth fusion with TVL2 successfully computed and stored at " << tvl2Dir << endl;
    }

    time2 = getTickCount();
    time = (time2 - time1)/getTickFrequency();
    if(COMPUTE_TVL2) cout << "\nComputing depth fusion with TVL2 took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;

    if (!COMPUTE_TVL1) return EXIT_SUCCESS;

    // *****************************************************
    // **************** TVL1 OPTIMIZATION ******************
    // *****************************************************

    time1 = getTickCount();

    if(COMPUTE_TVL1){

        cout << "\nComputing depth fusion with TVL1..." << endl;

        cout << "\nYou called:\n\n"
             << "	--lambda  <lambda>                           " << lambda << "\n"
             << "	--theta   <theta>                            " << theta << "\n"
             << "	--tau     <tau>                              " << tau << "\n"
             << "	--maxit   <maximum iterations>               " << maxIt << endl;
    }

    Mat tvl1;

    if(COMPUTE_TVL1){
        // Initialize the output implicit surface resulting from the depth fusion
        Mat_<float> u;
        if(TVL2_TVL1){
            u = tvl2.clone();
        }
        else {
            u = Mat::ones(1, bbox.nvoxels, CV_32FC1);
        }
        // Compute depth fusion optimizing TVL1 energy functional
        solveTVL1(u, &tvl1, bbox, distancefield, lambda, theta, maxIt, tau, tolerance, tvl1Dir, Icolor, P, DEBUG);
        cout << "\nDepth fusion with TVL1 successfully computed and stored at " << tvl1Dir << endl;
    }

    time2 = getTickCount();
    time = (time2 - time1)/getTickFrequency();
    if(COMPUTE_TVL1) cout << "\nComputing depth fusion with TVL1 took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;

    // *****************************************************
    // ***************** MAIN PROGRAM END ******************
    // *****************************************************

    return 0;
}
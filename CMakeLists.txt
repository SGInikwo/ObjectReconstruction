cmake_minimum_required(VERSION 3.17)
project(objRecon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/qt/5.15.2")
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/vtk@8.2/8.2.0_2")
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/opencv@3/3.4.13_2")

set(PCL_COMMON_INCLUDE_DIR "/usr/local/include/pcl-1.11/pcl")
set(PCL_DIR "/usr/local/share/pcl-1.11")
set(PCL_IO_INCLUDE_DIR "/usr/local/include/pcl-1.11/pcl")
set(PCL_OCTREE_INCLUDE_DIR "/usr/local/include/pcl-1.11/pcl")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message("Found OpenCV")
endif(OpenCV_FOUND)

set(CMAKE_C_COMPILER "/usr/local/Cellar/llvm/12.0.0/bin/clang")
set(CMAKE_CXX_COMPILER "/usr/local/Cellar/llvm/12.0.0/bin/clang++")
set(OPENMP_LIBRARIES "/usr/local/Cellar/llvm/12.0.0/lib")
set(OPENMP_INCLUDES "/usr/local/Cellar/llvm/12.0.0/include")

OPTION (USE_OpenMP "Use OpenMP to enable <omp.h>" ON)

# Find OpenMP
if(APPLE AND USE_OpenMP)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
        set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
endif()

if(USE_OpenMP)
    find_package(OpenMP REQUIRED)
endif(USE_OpenMP)

if (OPENMP_FOUND)
    include_directories("${OPENMP_INCLUDES}")
    link_directories("${OPENMP_LIBRARIES}")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

find_package(PCL REQUIRED)
if(PCL_FOUND)
    message("Found PCL")
endif(PCL_FOUND)

set(VTK_DIR "/usr/local/Cellar/vtk@8.2/8.2.0_2/lib/cmake/vtk-8.2")
find_package(VTK REQUIRED)
if(VTK_FOUND)
    message("Found VTK. Version:" ${VTK_VERSION})
endif(VTK_FOUND)


find_package(wxWidgets REQUIRED gl core base OPTIONAL_COMPONENTS net)

include(${wxWidgets_USE_FILE})
include(${VTK_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(objRecon rApp.cpp rMain.cpp AdjustBox.cpp Rec3D.cpp MarchingCubes.cpp camera.cpp bbox.cpp pointcloud.cpp planesweep.cpp tvl1.cpp)
target_link_libraries(objRecon ${wxWidgets_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES})
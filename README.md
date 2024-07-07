# PCL_Autonomous_Driving
 
 Autonomous Driving Project
This project involves the processing of point cloud data for autonomous driving applications. The key functionalities include loading point clouds, registering point clouds, segmenting road and objects, extracting features, and classifying the data. This README provides an overview of the project structure, setup instructions, and usage guidelines.

Directory Structure

├── build-cmake/                  # Directory for CMake build files
│   ├── Release/                  # Directory for Release build artifacts
│   └── Debug/                    # Directory for Debug build artifacts
├── data/                         # Directory for point cloud data files
│   ├── office1.pcd               # Example point cloud file
│   ├── office2.pcd               # Example point cloud file
│   ├── registered.pcd            # Output file for registered point cloud
│   ├── road.pcd                  # Output file for segmented road
│   └── object_*.pcd              # Output files for detected objects
├── include/                      # Header files for the project
│   └── point_cloud_processing.h  # Header file for point cloud processing class
├── src/                          # Source files for the project
│   ├── main.cpp                  # Main entry point of the project
│   └── point_cloud_processing.cpp # Implementation of point cloud processing class
├── CMakeLists.txt                # CMake build configuration file
└── README.md                     # This README file

Prerequisites
- CMake version 3.10 or higher
- vcpkg for managing dependencies
- PCL (Point Cloud Library) installed via vcpkg
1. Setup Instructions
Clone the repository

git clone https://github.com/yourusername/autonomous-driving-project.git
cd autonomous-driving-project

2. Install dependencies

Ensure you have vcpkg installed and initialized. Install PCL using vcpkg:

./vcpkg install pcl

3. Configure and build the project

Create a build directory and run CMake:

mkdir build-cmake
cd build-cmake
cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release

Running the Project

1. Prepare your data

Place your input PCD files (e.g., office1.pcd, office2.pcd) in the data/ directory.

2. Execute the application

Navigate to the build directory and run the executable:

cd build-cmake/Release
./PCL_Autonomous_Driving_Project

Ensure the data/ directory is in the correct path relative to the executable, or update the file paths in the code accordingly.

- Project Functionalities
Load Point Cloud: Loads point cloud data from PCD files.
Register Point Clouds: Aligns and merges two point clouds.
Segment Road: Segments the road surface from the point cloud.
Detect Objects: Detects and extracts objects from the point cloud.
Extract Features: Computes feature descriptors for the point cloud.
Classify Point Cloud: Classifies the point cloud data based on extracted features.
- Example Usage
In the main.cpp file, the following steps are performed:

Load the source and target point clouds.
Register the point clouds and save the registered output.
Segment the road from the registered point cloud and save the output.
Detect objects in the registered point cloud and save each detected object.
Extract features from the registered and target point clouds.
Classify the point clouds based on the extracted features.
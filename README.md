# Autonomous Driving Project

This project involves the processing of point cloud data for autonomous driving applications. The key functionalities include loading point clouds, registering point clouds, segmenting road and objects, extracting features, and classifying the data. This README provides an overview of the project structure, setup instructions, and usage guidelines.


## Prerequisites

- [CMake](https://cmake.org/) version 3.10 or higher
- [vcpkg](https://github.com/microsoft/vcpkg) for managing dependencies
- PCL (Point Cloud Library) installed via vcpkg

## Setup Instructions

1. **Clone the repository**

    ```bash
    git clone https://github.com/yourusername/autonomous-driving-project.git
    cd autonomous-driving-project
    ```

2. **Install dependencies**

    Ensure you have `vcpkg` installed and initialized. Install PCL using `vcpkg`:

    ```bash
    ./vcpkg install pcl
    ```

3. **Configure and build the project**

    Create a build directory and run CMake:

    ```bash
    mkdir build-cmake
    cd build-cmake
    cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
    cmake --build . --config Release
    ```

## Running the Project

1. **Prepare your data**

    Place your input PCD files (e.g., `office1.pcd`, `office2.pcd`) in the `data/` directory.

2. **Execute the application**

    Navigate to the build directory and run the executable:

    ```bash
    cd build-cmake/Release
    ./PCL_Autonomous_Driving_Project
    ```

    Ensure the `data/` directory is in the correct path relative to the executable, or update the file paths in the code accordingly.

## Project Functionalities

- **Load Point Cloud**: Loads point cloud data from PCD files.
- **Register Point Clouds**: Aligns and merges two point clouds.
- **Segment Road**: Segments the road surface from the point cloud.
- **Detect Objects**: Detects and extracts objects from the point cloud.
- **Extract Features**: Computes feature descriptors for the point cloud.
- **Classify Point Cloud**: Classifies the point cloud data based on extracted features.

## Example Usage

In the `main.cpp` file, the following steps are performed:

1. Load the source and target point clouds.
2. Register the point clouds and save the registered output.
3. Segment the road from the registered point cloud and save the output.
4. Detect objects in the registered point cloud and save each detected object.
5. Extract features from the registered and target point clouds.
6. Classify the point clouds based on the extracted features.


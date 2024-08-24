
# Lab 2 - 3D Obstacle Detection from Point Clouds

**The full assignment instruction pdf is [here](docs/Lab2.pdf)**.

## Overview

This folder contains the implementation and results for Lab 2, which focuses on 3D obstacle detection using point cloud data. The lab explores various techniques for filtering and clustering point clouds to detect obstacles, with a particular emphasis on computational efficiency and real-time processing.


## Objectives

The main objectives of this lab are:
- Understanding the concept and structure of 3D point clouds.
- Implementing ground point removal to filter out irrelevant data.
- Developing algorithms to filter points by distance for effective obstacle detection.
- Applying the Euclidean clustering algorithm to group detected points into distinct obstacles.
- Computing bounding boxes around detected obstacles for visualization.
- Enhancing the performance of the clustering algorithm to enable real-time processing on robotic systems.
- Exploring additional improvements to the algorithm for better performance and accuracy.

#### Data:
Two recorded 3d lidar Rosbag files. Download from [here](https://drive.google.com/drive/folders/1kFWmUvjs5I9GQEOAF9Qm00MaSwgtn0Hv?usp=sharing) and put inside [ai_labs/data/](ai_labs/data/)

## Implementation Details
**The full source code found inside [ai_labs](./ai_labs/)**.


### 1. Ground Point Filtering (`filter_ground`)
This function removes points that are on the ground surface by applying a height threshold to the Z-coordinate of the point cloud data. Points with a Z-coordinate below the specified threshold (e.g., 1 meter) are filtered out. The Z-coordinate is relative to the center of the laser scanner.

**Output**: The filtered point cloud data is visualized in the attached video.

### 2. Distance-Based Filtering (`filter_by_distance`)
This function filters out points that are beyond a certain horizontal distance from the vehicle. By setting a horizontal distance threshold (e.g., 10 meters), only points within this range are considered for obstacle detection. The horizontal distance is calculated using the X and Y coordinates of each point in the cloud.

**Output**: The filtered point cloud data based on distance is visualized in the attached video.

### 3. Euclidean Clustering (`euclidean_clustering`)
The Euclidean clustering algorithm organizes the filtered point cloud data into clusters that represent individual obstacles. The algorithm groups points together based on a specified distance threshold (e.g., 0.5 meters), resulting in distinct clusters for each obstacle. Scikit-learn's KDTree is used to efficiently find neighboring points, which speeds up the clustering process.

**Output**: The clustered point cloud data is visualized in the attached video.

### 4. Bounding Box Computation (`get_bounding_boxes`)
This function computes the bounding boxes for each detected obstacle by determining the minimum and maximum coordinates in the X, Y, and Z dimensions for each cluster. These bounding boxes are visualized in RVIZ, providing a clear visual representation of the detected obstacles.

**Output**: The bounding boxes are visualized in RVIZ, as shown in the attached video.

## Extra Credit

### 5. Running on a Second Dataset
The obstacle detection code is tested on a second dataset (`kitti_2011_09_26_drive_0002_synced.bag`). Parameters such as the ground level threshold and distance threshold were adjusted to accommodate the new dataset.

**Output**: The results are visualized in the attached video.

### 6. Filtering Clusters by Size (`filter_clusters`)
This function addresses the issue of noisy point cloud data by filtering out clusters that are too small, which could be falsely detected as obstacles. Clusters with fewer than a specified number of points (e.g., 100 points) are removed, and these points are assigned a cluster label of 0.

**Output**: The filtered clusters are visualized in the attached video.

### 7. Accelerated Euclidean Clustering (`euclidean_clustering_accelerated`)
To improve the computational efficiency of the Euclidean clustering algorithm, the FLANN library is used to accelerate the process of finding neighboring points. The algorithm is optimized using NumPy arrays and vectorized operations, which reduces the computation time significantly (from approximately 2.2 seconds to 0.2 seconds).

**Output**: A comparison of the computation times between the original and accelerated algorithms is provided, demonstrating the improved performance.

### 8. Open-Ended Improvements
Further improvements were made to the Euclidean clustering algorithm by using KDTree for indexing point cloud points and applying vectorization techniques. These optimizations further reduced the computational complexity from \(O(n^3)\) to \(O(n^2)\).

**Output**: The optimized algorithm's performance is demonstrated in the attached video.

## Conclusion

In this lab I successfully implemented and tested various techniques for 3D obstacle detection using point clouds. The optimized Euclidean clustering algorithm, combined with effective filtering techniques, enables real-time processing, making it suitable for robotic applications. The additional enhancements further improved the algorithm's efficiency and accuracy, demonstrating the potential for real-world deployment.

## References
- [FLANN Library](https://pypi.org/project/flann/)
- [GitHub Issue on FLANN with Python3](https://github.com/primetang/pyflann/issues/1)

**The full solution pdf found in [here](Lab2_Ai_robotics.pdf).**

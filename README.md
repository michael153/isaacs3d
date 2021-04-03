# ISAACS 2.5D Research Project

## Overview

This project plans drone missions by computing the waypoints necessary for radiation source detection among shipping containers
in a previously unseen environment. The waypoints are computed by generating raster paths over the shipping containers' surfaces, which are algorithmically computed from the drone's sensory point cloud data. 

The actual radiation source detection given the data collected over the raster paths is implemented separately and follows the implementation described in
[A Successive-Elimination Approach to Adaptive Robotic Sensing](https://arxiv.org/abs/1809.10611).

<p align="center">
  <img src="images/example_pointcloud_data.png" width="785" alt="">
</p>

<p align="center">
  <img src="images/example_container_extraction.png" width="250" alt="">&nbsp;&nbsp;&nbsp;
  <img src="images/example_surface_reconstruction.png" width="250" alt="">&nbsp;&nbsp;&nbsp;
  <img src="images/example_path_generation.png" width="250" alt="">
</p>
<p align="center">
  Example container extraction, surface reconstruction, and path generation from the point cloud data
</div>

## Getting Started

### Setup 

#### 1. Install pip requirements
```pip3 install -r requirements.txt```

#### 2. Set file paths
You can set the path to the input point cloud file (must be a .ply file) and the path to the output waypoints path (must be a .pkl file) by running
```python3 set_paths.py -in [path to point cloud file] -out [path to output waypoints path]```

### Running the path planner

#### 3. Execution
```python3 container_extraction.py [--verbose] [--graph]```
The flags specify whether you want the planner to print out debugging statements and if you want to graph the container extraction, surface reconstruction, and path generation.

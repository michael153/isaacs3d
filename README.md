# ISAACS 2.5D Research Project

This project plans drone missions by computing the waypoints necessary for radiation source detection among shipping containers
in a previously unseen environment. The waypoints are computed by generating raster paths over the shipping containers' surfaces, which are algorithmically computed from the drone's sensory point cloud data. 

The actual radiation source detection given the data collected over the raster paths is implemented separately and follows the implementation described in
[A Successive-Elimination Approach to Adaptive Robotic Sensing](https://arxiv.org/abs/1809.10611).


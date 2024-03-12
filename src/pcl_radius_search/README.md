## PCL_Radius_Search

### Enviroment

- Ubuntu 18.04
- PCL 1.14

### Description

This package requires converting ROS point cloud messages to PCL for processing. Typically, the pcl_conversions package can handle this task. However, it relies on PCL 1.8, which may conflict with other libraries dependent on PCL 1.14. 

Therefore, in this case, the conversion process is replaced with a custom conversion function.
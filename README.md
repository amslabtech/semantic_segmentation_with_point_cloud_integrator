# semantic_segmentation_with_point_cloud_integrator
Integrates image semantic segmentation with point cloud

## Environment
- Ubuntu 16.04 or 18.04
- ROS kinetic or melodic
- PCL(>=1.8.0)

## Requirement
- tf from image sensor to point cloud sensor is required

## Node
### semantic_segmentation_with_point_cloud_integrator
- Published topics
  - /cloud/colored/raw (sensor_msgs/PointCloud2)
    - Point cloud colored to the color of the corresponding pixel in the semantic segmented image
  - /cloud/colored/semantic (sensor_msgs/PointCloud2)
    - Point cloud colored to the color of the corresponding pixel in the semantic segmented image (specific classes only)
  - /projection/raw (sensor_msgs/Image)
    - An image in which point clouds colored by its distance
  - /projection/semantic (sensor_msgs/Image)
    - An image in which point clouds colored with the corresponding pixel color are plotted
- Subscribed topics
  - /segmented_image (sensor_msgs/Image)
    - Color-coded images with semantic segmentation (e.g. deeplab)
  - /camera_info (sensor_msgs/Image)
  - /pointcloud (sensor_msgs/PointCloud2)

## Install and Build
```
cd your_workspace
git clone https://github.com/amslabtech/semantic_segmentation_with_point_cloud_integrator.git
catkin build
```

## Notes
- Currently only Cityscapes dataset is supported

#ifndef __SEMANTIC_SEGMENTATION_WITH_POINT_CLOUD_INTEGRATOR_H
#define __SEMANTIC_SEGMENTATION_WITH_POINT_CLOUD_INTEGRATOR_H

#include <omp.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class SemanticSegmentationWithPointCloudIntegrator
{
public:
    typedef pcl::PointXYZRGB PointType ;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef PointCloudType::Ptr PointCloudTypePtr;

    SemanticSegmentationWithPointCloudIntegrator(void);

    void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&, const sensor_msgs::PointCloud2ConstPtr&);
    void get_color_from_distance(double, int&, int&, int&);
    void sensor_fusion(const sensor_msgs::Image&, const sensor_msgs::CameraInfo&, const sensor_msgs::PointCloud2&);
    void coloring_pointcloud(PointCloudTypePtr&, int, int, int);
    bool is_in_extraction_classes(int, int, int);
    void process(void);

private:
    double MAX_DISTANCE;
    std::string LABELS_PATH;
    std::string EXTRACTION_CLASSES;
    std::vector<std::string> EXTRACTION_CLASSES_LIST;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sensor_fusion_sync_subs;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Synchronizer<sensor_fusion_sync_subs> sensor_fusion_sync;

    ros::Publisher image_pub;
    ros::Publisher pc_pub;
    ros::Publisher semantic_cloud_pub;
    ros::Publisher projection_semantic_image_pub;

    tf::TransformListener listener;
    Eigen::Affine3d transform;
    typedef std::tuple<int, int, int> ColorTuple;
    std::map<ColorTuple, std::string> color_with_class;
};

#endif// __SEMANTIC_SEGMENTATION_WITH_POINT_CLOUD_INTEGRATOR_H

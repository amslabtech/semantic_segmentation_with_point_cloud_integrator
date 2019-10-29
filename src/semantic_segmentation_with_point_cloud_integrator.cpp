#include "semantic_segmentation_with_point_cloud_integrator/semantic_segmentation_with_point_cloud_integrator.h"

SemanticSegmentationWithPointCloudIntegrator::SemanticSegmentationWithPointCloudIntegrator(void)
    : local_nh("~"),
      image_sub(nh, "/deeplab/segmented_image", 10), camera_info_sub(nh, "/camera_info", 10), pc_sub(nh, "/pointcloud", 10), sensor_fusion_sync(sensor_fusion_sync_subs(10), image_sub, camera_info_sub, pc_sub)
{
    image_pub = nh.advertise<sensor_msgs::Image>("/projection/raw", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/colored/raw", 1);
    semantic_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/colored/semantic", 1);
    projection_semantic_image_pub = nh.advertise<sensor_msgs::Image>("/projection/semantic", 1);
    sensor_fusion_sync.registerCallback(boost::bind(&SemanticSegmentationWithPointCloudIntegrator::callback, this, _1, _2, _3));

    local_nh.param("MAX_DISTANCE", MAX_DISTANCE, {20.0});
    local_nh.param("LABELS_PATH", LABELS_PATH, {""});
    local_nh.param("EXTRACTION_CLASSES", EXTRACTION_CLASSES, {"ground, road, sidewalk, terrain"});

    std::cout << "=== semantic_segmentation_with_point_cloud_integrator ===" << std::endl;
    std::cout << "MAX_DISTANCE: " << MAX_DISTANCE << std::endl;
    std::cout << "LABELS_PATH: " << LABELS_PATH << std::endl;
    std::cout << "EXTRACTION_CLASSES: " << std::endl;

    // remove space
    size_t pos;
    while((pos = EXTRACTION_CLASSES.find_first_of(" 　\t")) != std::string::npos){
        EXTRACTION_CLASSES.erase(pos, 1);
    }
    // separate by ","
    while(1){
        static auto offset = std::string::size_type(0);
        auto pos = EXTRACTION_CLASSES.find(",", offset);
        if(pos == std::string::npos){
            EXTRACTION_CLASSES_LIST.push_back(EXTRACTION_CLASSES.substr(offset));
            break;
        }
        EXTRACTION_CLASSES_LIST.push_back(EXTRACTION_CLASSES.substr(offset, pos - offset));
        offset = pos + std::string(",").length();
    }
    for(const auto& str : EXTRACTION_CLASSES_LIST){
        std::cout << str << std::endl;
    }

    // load json
    boost::property_tree::ptree ptree;
    boost::property_tree::read_json(LABELS_PATH, ptree);

    BOOST_FOREACH(boost::property_tree::ptree::value_type &child, ptree.get_child("labels"))
    {
        std::string class_name = "";
        int red = 0;
        int green = 0;
        int blue = 0;
        const boost::property_tree::ptree& labels = child.second;
        if(boost::optional<std::string> label = labels.get_optional<std::string>("label")) {
            std::cout << "label : " << label.get() << std::endl;
            class_name = label.get();
        }else{
            std::cout << "label is nothing" << std::endl;
            exit(-1);
        }
        if(std::find(EXTRACTION_CLASSES_LIST.begin(), EXTRACTION_CLASSES_LIST.end(), class_name) == EXTRACTION_CLASSES_LIST.end()){
            std::cout << "skipped" << std::endl;
            continue;
        }
        const boost::property_tree::ptree& color = child.second.get_child("color");
        if(boost::optional<int> r = color.get_optional<int>("r")){
            std::cout << "r : " << r.get() << std::endl;
            red = r.get();
        }else{
            std::cout << "color.r is nothing" << std::endl;
            exit(-1);
        }
        if(boost::optional<int> g = color.get_optional<int>("g")){
            std::cout << "g : " << g.get() << std::endl;
            green = g.get();
        }else{
            std::cout << "color.g is nothing" << std::endl;
            exit(-1);
        }
        if(boost::optional<int> b = color.get_optional<int>("b")){
            std::cout << "b : " << b.get() << std::endl;
            blue = b.get();
        }else{
            std::cout << "color.b is nothing" << std::endl;
            exit(-1);
        }
        std::cout << red << ", " << green << ", " << blue << ", " << class_name << std::endl;;
        color_with_class[std::make_tuple(red, green, blue)] = class_name;
    }
    std::cout << "json was loaded" << std::endl;
    std::cout << "color with class:" << std::endl;
    std::cout << "size: " << color_with_class.size() << std::endl;
    for(const auto& value : color_with_class){
        std::cout << "red: " << std::get<0>(value.first) << ", green:" << std::get<1>(value.first) << ", blue: " << std::get<2>(value.first) << ", class: " << value.second << std::endl;
    }
    std::cout << "waiting for data..." << std::endl;
}

void SemanticSegmentationWithPointCloudIntegrator::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info, const sensor_msgs::PointCloud2ConstPtr& pc)
{
    std::cout << "=== semantic_segmentation_with_point_cloud_integrator ===" << std::endl;
    try{
        tf::StampedTransform transform_;
        listener.waitForTransform(image->header.frame_id, pc->header.frame_id, ros::Time(0), ros::Duration(4.0));
        listener.lookupTransform(image->header.frame_id, pc->header.frame_id, ros::Time(0), transform_);
        tf::transformTFToEigen(transform_, transform);
        sensor_fusion(*image, *camera_info, *pc);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
}

void SemanticSegmentationWithPointCloudIntegrator::get_color_from_distance(double distance, int& r, int& g, int& b)
{
    r = 255;
    g = 255;
    b = 255;

    distance = std::max(std::min(distance, MAX_DISTANCE), 0.0);

    double v = distance / MAX_DISTANCE * 255;// 0 ~ 255

    if(v < (0.25 * 255)){
        r = 0;
        g = 4 * v;
    }else if(v < 0.5 * 255){
        r = 0;
        b = 255 + 4 * (0.25 * 255 - v);
    }else if(v < 0.75 * 255){
        r = 4 * (v - 0.5 * 255);
        b = 0;
    }else{
        g = 255 + 4 * (0.75 * 255 - v);
        b = 0;
    }
}

void SemanticSegmentationWithPointCloudIntegrator::sensor_fusion(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info, const sensor_msgs::PointCloud2& pc)
{
    double start_time = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc, *lidar_cloud);

    PointCloudTypePtr cloud(new PointCloudType);
    pcl::copyPointCloud(*lidar_cloud, *cloud);

    PointCloudTypePtr trans_cloud(new PointCloudType);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception& ex){
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;

    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info);

    PointCloudTypePtr colored_cloud(new PointCloudType);
    *colored_cloud = *trans_cloud;
    cv::Mat projection_image = rgb_image.clone();

    // semantic cloud
    PointCloudTypePtr semantic_cloud(new PointCloudType);
    semantic_cloud->header = colored_cloud->header;

    for(auto& pt : colored_cloud->points){
        if(pt.z<0){
            // behind camera
            pt.b = 255;
            pt.g = 255;
            pt.r = 255;
        }else{
            cv::Point3d pt_cv(pt.x, pt.y, pt.z);
            cv::Point2d uv;
            uv = cam_model.project3dToPixel(pt_cv);

            if(uv.x > 0 && uv. x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows){
                pt.b = rgb_image.at<cv::Vec3b>(uv)[0];
                pt.g = rgb_image.at<cv::Vec3b>(uv)[1];
                pt.r = rgb_image.at<cv::Vec3b>(uv)[2];

                if(is_in_extraction_classes(pt.r, pt.g, pt.b)){
                    semantic_cloud->points.push_back(pt);
                }

                double distance = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                int r, g, b;
                get_color_from_distance(distance, r, g, b);
                cv::circle(projection_image, uv, 1, cv::Scalar(b, g, r), -1);
            }else{
                pt.b = 255;
                pt.g = 255;
                pt.r = 255;
            }
        }
    }

    PointCloudTypePtr output_sc(new PointCloudType);
    pcl::copyPointCloud(*semantic_cloud, *output_sc);
    pcl::transformPointCloud(*output_sc, *output_sc, transform.inverse());
    sensor_msgs::PointCloud2 output_semantic_cloud;
    pcl::toROSMsg(*output_sc, output_semantic_cloud);
    output_semantic_cloud.header = pc.header;
    semantic_cloud_pub.publish(output_semantic_cloud);

    // projection/semantic image
    cv::Mat projection_semantic_image;
    projection_image.copyTo(projection_semantic_image);
    for(const auto& pt : semantic_cloud->points){
        cv::Point3d pt_cv(pt.x, pt.y, pt.z);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        cv::circle(projection_semantic_image, uv, 3, cv::Scalar(pt.b, pt.g, pt.r), -1);
    }
    sensor_msgs::ImagePtr output_psi;
    output_psi = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_semantic_image).toImageMsg();
    output_psi->header = image.header;
    projection_semantic_image_pub.publish(output_psi);

    // colored cloud
    PointCloudTypePtr output_cloud(new PointCloudType);
    pcl::transformPointCloud(*colored_cloud, *output_cloud, transform.inverse());
    sensor_msgs::PointCloud2 output_pc;
    pcl::toROSMsg(*output_cloud, output_pc);
    output_pc.header = pc.header;
    pc_pub.publish(output_pc);

    // projection/depth image
    sensor_msgs::ImagePtr output_image;
    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_image).toImageMsg();
    output_image->header = image.header;
    image_pub.publish(output_image);

    std::cout << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void SemanticSegmentationWithPointCloudIntegrator::coloring_pointcloud(PointCloudTypePtr& cloud, int r, int g, int b)
{
    for(auto& pt : cloud->points){
        pt.r = r;
        pt.g = g;
        pt.b = b;
    }
}

bool SemanticSegmentationWithPointCloudIntegrator::is_in_extraction_classes(int r, int g, int b)
{
    auto color_tuple = std::make_tuple(r, g, b);
    for(const auto& value : color_with_class){
        if(color_tuple == value.first){
            return true;
        }
    }
    return false;
}

void SemanticSegmentationWithPointCloudIntegrator::process(void)
{
    ros::spin();
}

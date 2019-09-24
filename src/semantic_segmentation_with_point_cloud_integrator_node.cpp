#include "semantic_segmentation_with_point_cloud_integrator/semantic_segmentation_with_point_cloud_integrator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation_with_point_cloud_integrator");
    SemanticSegmentationWithPointCloudIntegrator sswpci;
    sswpci.process();
    return 0;
}

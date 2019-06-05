#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub, pub2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud_blob = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudBlobPtr(cloud_blob);
    pcl::PCLPointCloud2 cloud_filtered_blob;
    // pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud_blob);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudBlobPtr);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(cloud_filtered_blob);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered_blob, output);
    pub.publish(output);

    // Convert the PCL data to pcl/PointCloud
    pcl::fromPCLPointCloud2(cloud_filtered_blob, *cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg(*cloud_p, output2);
        // pcl_conversions::fromPCL(*cloud_p, output);
        pub2.publish(output2);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("output2", 1);

    // Spin
    ros::spin();
}
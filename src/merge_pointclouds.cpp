#include <iostream>
#include <string>

#include <ros/ros.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const std::string LOGNAME = "merge_pointclouds";


auto filter(const PointCloudT::ConstPtr &points) {
    auto points_filtered = boost::make_shared<PointCloudT>();

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(points);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);
    outrem.setKeepOrganized(true);
    outrem.filter(*points_filtered);

    return points_filtered;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "merge_pointclouds");

    ros::NodeHandle nh;

    auto const pub = nh.advertise<sensor_msgs::PointCloud2>("merged_points", 1);

    auto callback = [&](const sensor_msgs::PointCloud2ConstPtr &points1_msg,
                        const sensor_msgs::PointCloud2ConstPtr &points2_msg) -> void {

        pcl::PCLPointCloud2 points1_v2;
        pcl::PCLPointCloud2 points2_v2;
        auto points1 = boost::make_shared<PointCloudT>();
        auto points2 = boost::make_shared<PointCloudT>();
        PointCloudT points2_icp;

        pcl_conversions::toPCL(*points1_msg, points1_v2);
        pcl_conversions::toPCL(*points2_msg, points2_v2);

        pcl::fromPCLPointCloud2(points1_v2, *points1);
        pcl::fromPCLPointCloud2(points2_v2, *points2);

        const auto points1_filtered = filter(points1);
        const auto points2_filtered = filter(points2);

        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

        // The Iterative Closest Point algorithm
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setMaximumIterations(1);
        icp.setInputSource(points1);
        icp.setInputTarget(points2);
        icp.align(points2_icp);
        icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function

        if (icp.hasConverged()) {
            ROS_DEBUG_STREAM_NAMED(LOGNAME, "ICP has converged, score is " << icp.getFitnessScore());
            const auto matrix = icp.getFinalTransformation().cast<double>();
            ROS_DEBUG_STREAM_NAMED(LOGNAME, "Transformation:\n" << matrix);
        } else {
            ROS_ERROR_NAMED(LOGNAME, "ICP did not converge");
        }

        const auto merged_points = *points1 + points2_icp;
        pcl::PCLPointCloud2 merged_points_v2;
        pcl::toPCLPointCloud2(merged_points, merged_points_v2);

        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(merged_points_v2, output);

        pub.publish(output);
    };

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "points1", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "points2", 1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind<void>(callback, _1, _2));

    ros::spin();

    return EXIT_SUCCESS;
}


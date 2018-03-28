#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <iostream>
using namespace std;

ros::Publisher pub_nocar_points;

void callback(const sensor_msgs:: PointCloud2ConstPtr& ground_points_msg,
              const sensor_msgs:: PointCloud2ConstPtr& nocarground_points_msg)
{
    double ground_points_time = ground_points_msg->header.stamp.toSec();
    double nocarground_points_time = nocarground_points_msg->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_points(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 ground = *ground_points_msg;
    sensor_msgs::PointCloud2 nocarground = *nocarground_points_msg;

    sensor_msgs::PointCloud2Iterator<float> ground_x(ground , "x");
    sensor_msgs::PointCloud2Iterator<float> ground_y(ground , "y");
    sensor_msgs::PointCloud2Iterator<float> ground_z(ground , "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> ground_i(ground , "intensity"); //uint8_t for rslidar, float for velodyne
    for (int i = 0; ground_z != ground_z.end(); ++i, ++ground_x, ++ground_y, ++ground_z, ++ground_i)
    {
        pcl::PointXYZI point;
        point.x = (*ground_x);
        point.y = (*ground_y);
        point.z = (*ground_z);     
        point.intensity = unsigned(*ground_i);
        combined_points->points.push_back(point);
    }
    std::cout<<"1: "<< combined_points->points.size()<<endl;
    sensor_msgs::PointCloud2Iterator<float> nocarground_x(nocarground , "x");
    sensor_msgs::PointCloud2Iterator<float> nocarground_y(nocarground , "y");
    sensor_msgs::PointCloud2Iterator<float> nocarground_z(nocarground , "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> nocarground_i(nocarground , "intensity"); 
    for (int i = 0; nocarground_z != nocarground_z.end(); ++i, ++nocarground_x, ++nocarground_y, ++nocarground_z, ++nocarground_i)
    {
        pcl::PointXYZI point;
        point.x = (*nocarground_x);
        point.y = (*nocarground_y);
        point.z = (*nocarground_z);     
        point.intensity = unsigned(*nocarground_i);
        combined_points->points.push_back(point);
    }
    std::cout<<"2: "<< combined_points->points.size()<<endl;
    
    sensor_msgs::PointCloud2 nocar_points_msg;
    pcl::toROSMsg(*combined_points, nocar_points_msg);
    nocar_points_msg.header.stamp=ros::Time().fromSec(ground_points_time);
    nocar_points_msg.header.frame_id = "pandar";
    pub_nocar_points.publish(nocar_points_msg);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "combine_data");
    ros::NodeHandle nh;
    std::string GROUND = "ground_points";
    std::string NOCARGROUND = "nocarground_points";

    message_filters::Subscriber<sensor_msgs::PointCloud2> ground_points(nh, GROUND, 1);//订阅16线激光雷达Topic
    message_filters::Subscriber<sensor_msgs::PointCloud2> nocarground_points(nh, NOCARGROUND, 1);//订阅16线激光雷达Topic
    pub_nocar_points = nh.advertise<sensor_msgs::PointCloud2>("nocar_points", 1);

    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), plane_points, nodynamic_points);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(ground_points, nocarground_points, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
   
    ros::spin();
    return 0;
}
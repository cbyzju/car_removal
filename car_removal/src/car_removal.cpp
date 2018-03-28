#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

ros::Publisher pub_nocarground_points;

inline bool inside(const Rect& small, Rect& big)
{
    return ((small.x > big.x && small.x< big.x + big.width && small.y > big.y && small.y < big.y + big.height) || 
            (small.x +small.width > big.x && small.x + small.width < big.x + big.width && small.y + small.height > big.y && small.y+ small.height < big.y + big.height));
}

inline bool inside(const Point& point, const Rect& region)
{
    return (point.x >= region.x - 5 && point.x <= region.x + region.width + 10 &&
            point.y >= region.y - 5 && point.y <= region.y + region.height + 10);
}

void removalCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
    double lidar_time = lidar_msg->header.stamp.toSec();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::fromROSMsg(*lidar_msg, *laserCloudIn);
    cv::Mat projection(501, 501, CV_8UC1, cv::Scalar(0));
    float resolution = 0.1;
    sensor_msgs::PointCloud2 lidar = *lidar_msg;
    sensor_msgs::PointCloud2Iterator<float> iter_x(lidar, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(lidar, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(lidar, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_i(lidar, "intensity"); //uint8_t for rslidar, float for velodyne
    for (int i = 0; iter_z != iter_z.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        pcl::PointXYZI point;
        point.x = (*iter_x);
        point.y = (*iter_y);
        point.z = (*iter_z);     
        point.intensity = unsigned(*iter_i);
        laserCloudIn->points.push_back(point);

        if(point.x<=-25 || point.x>=25 || point.y<=-25 || point.y>=25 || point.z>=5)
            continue;
        else
        {
            int x_index = round((point.x+25)/resolution);
            int y_index = round((point.y+25)/resolution);
            //cout<<"x_index = "<<x_index<<", y_index = "<<y_index<<endl;
            projection.at<uchar>(y_index, x_index) = 255;
        }
        
    }
    
    //Mat color_projection;
    //cvtColor(projection, color_projection, CV_GRAY2BGR);
    
    pcl::console::TicToc tt;
    tt.tic();
    GaussianBlur(projection, projection, Size(11,11), 0, 0);

    std::vector< std::vector<Point> > contours;
    findContours(projection, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //rectangle(color_projection, Point(130, 220), Point(130+15, 220+20), cv::Scalar(0,0,255), 3);

    std::vector<Rect> car_region;
    for(int i = 0; i<contours.size(); ++i)
    {
        //vector<Point> contour = contours[i];
		//Mat contourMat = Mat(contour);
		//double cArea = contourArea(contourMat);
        Rect rect = boundingRect(contours[i]);
        if((rect.width>10 && rect.width<40 && rect.height>10 && rect.height<60) ||
           (rect.width>10 && rect.width<60 && rect.height>10 && rect.height<40))
           {
                car_region.push_back(rect);
                //rectangle(color_projection, rect, cv::Scalar(0,255,0), 3);  
           }            
    }
   
   pcl::PointCloud<pcl::PointXYZI>::Ptr nocarground_points(new pcl::PointCloud<pcl::PointXYZI>);

   for(int i = 0; i<laserCloudIn->points.size(); ++i)
   {
       pcl::PointXYZI point;
       point.x = laserCloudIn->points[i].x;
       point.y = laserCloudIn->points[i].y;
       point.z = laserCloudIn->points[i].z;  
       point.intensity = laserCloudIn->points[i].intensity;
       bool pointoutside = true;

       Point tempPoint;
       tempPoint.x = round((point.x+25)/resolution);
       tempPoint.y = round((point.y+25)/resolution);
       for(int j = 0; j<car_region.size(); ++j)
       {
           if(inside(tempPoint, car_region[j]))
           {
               pointoutside = false;
               break;
           }            
       }
       if(pointoutside)
       {
               nocarground_points->points.push_back(point);
       }
            
   }
    cout<<"car removal cost "<< tt.toc()<<" ms"<<endl;
    sensor_msgs::PointCloud2 nocarground_points_msgs;
    pcl::toROSMsg(*nocarground_points, nocarground_points_msgs);
    nocarground_points_msgs.header.stamp=ros::Time().fromSec(lidar_time);
    nocarground_points_msgs.header.frame_id = "pandar";
    pub_nocarground_points.publish(nocarground_points_msgs);

    //imshow("projection", color_projection);
    //waitKey(1);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "car_removal");
    ros::NodeHandle nh;
    std::string TOPIC = "noground_points";

    ros::Subscriber point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(TOPIC, 1, removalCallback);
    pub_nocarground_points = nh.advertise<sensor_msgs::PointCloud2>("nocarground_points", 2);

    ros::spin();
    return 0;
}
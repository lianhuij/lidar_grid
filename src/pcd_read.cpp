#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#define pcd_file "/home/lhj/catkin_ws/src/data/road4_0515.pcd"
#define fixed_frame "velodyne"
#define pub_hz 10    //10Hz 发送频率

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcd_read");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1);  //发送话题：velodyne_points

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::io::loadPCDFile (pcd_file, cloud);   //从硬盘加载点云文件
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = fixed_frame;

    ros::Rate loop_rate(pub_hz);

    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
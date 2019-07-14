#include <ros/ros.h>
#include "/home/lhj/catkin_ws/devel/include/can_msgs/Frame.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

///////////////////////MPC单片机CAN消息处理类////////////////////////
class MPCDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_pub;
    ros::Publisher camera_pub;
    ros::Publisher fusion_pub;
    ros::Publisher obj_pub;
    std::string fixed_frame;
    float x_offset;

public:
    MPCDataHandler()
    {
        can_sub = nh.subscribe("received_messages", 10, &MPCDataHandler::canHandler, this);   //接收话题：received_messages
        radar_pub = nh.advertise<geometry_msgs::Point>("radar_pos", 1);                       //发布话题：radar_pos
        camera_pub = nh.advertise<geometry_msgs::Point>("camera_pos", 1);                     //发布话题：camera_pos
        fusion_pub = nh.advertise<geometry_msgs::Point>("fusion_pos", 1);                     //发布话题：fusion_pos
        obj_pub = nh.advertise<visualization_msgs::Marker>("nearest_obj", 1);                 //发布话题：nearest_obj

        nh.getParam("/MPC_data/fixed_frame", fixed_frame);
        nh.getParam("/MPC_data/x_offset", x_offset);
    }

    void canHandler(const can_msgs::Frame& input);
};

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void MPCDataHandler::canHandler(const can_msgs::Frame& input)
{
//////////////////////////////解析CAN消息///////////////////////////////

    if(input.id == 0x563)
    {
        geometry_msgs::Point radar_pos;
        geometry_msgs::Point camera_pos;

        radar_pos.x = (input.data[0]*256 + input.data[1])/100 + x_offset;
        radar_pos.y = (input.data[2]*256 + input.data[3])/10;
        camera_pos.x = (input.data[4]*256 + input.data[5])/100 + x_offset;
        camera_pos.y = (input.data[6]*256 + input.data[7])/16;

        radar_pub.publish(radar_pos);
        camera_pub.publish(camera_pos);
    }
    else if(input.id == 0x564)
    {
        geometry_msgs::Point fusion_pos;

        fusion_pos.x = (input.data[0]*256 + input.data[1])/100 + x_offset;
        fusion_pos.y = (input.data[2]*256 + input.data[3])/10;

        fusion_pub.publish(fusion_pos);

        visualization_msgs::Marker nearest;

        nearest.header.frame_id = fixed_frame;
        nearest.header.stamp = ros::Time::now();
        nearest.id = 0;
        nearest.type = visualization_msgs::Marker::CUBE;
        nearest.action = visualization_msgs::Marker::ADD;
        nearest.pose.position.x = fusion_pos.x;
        nearest.pose.position.y = fusion_pos.y;
        nearest.pose.position.z = -0.9;
        nearest.pose.orientation.x = 0.0;
        nearest.pose.orientation.y = 0.0;
        nearest.pose.orientation.z = 0.0;
        nearest.pose.orientation.w = 1.0;
        nearest.scale.x = 0.7;
        nearest.scale.y = 0.7;
        nearest.scale.z = 1.7;
        nearest.color.r = 0;
        nearest.color.g = 0;
        nearest.color.b = 0.5;
        nearest.color.a = 0.7;
        nearest.lifetime = ros::Duration();

        obj_pub.publish(nearest);
    }
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"MPC_data");

    MPCDataHandler handler;
        
    ros::spin();

    return 0;
}
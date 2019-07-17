#include <ros/ros.h>
#include "/home/lhj/catkin_ws/devel/include/can_msgs/Frame.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

typedef struct OBJECT {
    float x;
    float y;
} Objects;

///////////////////////MPC单片机CAN消息处理类////////////////////////
class MPCDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_pub;
    ros::Publisher camera_pub;
    ros::Publisher rawobj_pub;
    ros::Publisher fusion_pub;
    ros::Publisher obj_pub;
    std::string fixed_frame;
    float x_offset;

public:
    MPCDataHandler()
    {
        can_sub = nh.subscribe("received_messages", 1, &MPCDataHandler::canHandler, this);    //接收话题：received_messages
        radar_pub = nh.advertise<geometry_msgs::Point>("radar_pos", 1);                       //发布话题：radar_pos
        camera_pub = nh.advertise<geometry_msgs::Point>("camera_pos", 1);                     //发布话题：camera_pos
        rawobj_pub = nh.advertise<visualization_msgs::MarkerArray>("raw_objects", 1);         //发布话题：raw_objects
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
    int i;
//////////////////////////////解析CAN消息///////////////////////////////

    if(input.id == 0x563)
    {
        geometry_msgs::Point radar_pos;
        geometry_msgs::Point camera_pos;

        radar_pos.x = (input.data[0]*256 + input.data[1])/100;

        if(radar_pos.x > 0)
        {
            radar_pos.x = radar_pos.x + x_offset;
        }

        radar_pos.y = input.data[2]*256 + input.data[3];
        if(radar_pos.y < 0x8000)
        {
            radar_pos.y = radar_pos.y / 10;
        }
        else
        {
            radar_pos.y = (radar_pos.y - 0x10000)/10;
        }
        
        camera_pos.x = (input.data[4]*256 + input.data[5])/100;

        if(camera_pos.x > 0)
        {
            camera_pos.x = camera_pos.x + x_offset;
        }

        camera_pos.y = input.data[6]*256 + input.data[7];
        if(camera_pos.y < 0x8000)
        {
            camera_pos.y = camera_pos.y / 16;
        }
        else
        {
            camera_pos.y = (camera_pos.y - 0x10000)/16;
        }

        radar_pub.publish(radar_pos);     //发布毫米波雷达目标位置
        camera_pub.publish(camera_pos);   //发布摄像头目标位置

        visualization_msgs::MarkerArray objects;
        objects.markers.resize(2);
        std::vector<Objects> ped;
        ped.resize(2);
        ped[0].x = radar_pos.x;
        ped[0].y = radar_pos.y;
        ped[1].x = camera_pos.x;
        ped[1].y = camera_pos.y;

        for(i=0; i<2; ++i)
        {
            objects.markers[i].header.frame_id = fixed_frame;
            objects.markers[i].header.stamp = ros::Time::now();
            objects.markers[i].type = visualization_msgs::Marker::CYLINDER;
            objects.markers[i].action = visualization_msgs::Marker::ADD;
            objects.markers[i].id = i+1;
            objects.markers[i].pose.position.x = ped[i].x;
            objects.markers[i].pose.position.y = ped[i].y;
            objects.markers[i].pose.position.z = -0.9;
            objects.markers[i].pose.orientation.x = 0.0;
            objects.markers[i].pose.orientation.y = 0.0;
            objects.markers[i].pose.orientation.z = 0.0;
            objects.markers[i].pose.orientation.w = 1.0;
            objects.markers[i].scale.x = 0.6;
            objects.markers[i].scale.y = 0.6;
            objects.markers[i].scale.z = 1.7;

            if(i == 0)
            {
                objects.markers[i].color.r = 0;
                objects.markers[i].color.g = 0;
                objects.markers[i].color.b = 1;
            }
            else
            {
                objects.markers[i].color.r = 1;
                objects.markers[i].color.g = 0;
                objects.markers[i].color.b = 0;
            }
            
            if(ped[i].x == 0)
            {
                objects.markers[i].color.a = 0;
            }
            else
            {
                objects.markers[i].color.a = 0.7;
            }
            
            objects.markers[i].lifetime = ros::Duration();
        }

        rawobj_pub.publish(objects);
    }
    else if(input.id == 0x564)
    {
        geometry_msgs::Point fusion_pos;

        fusion_pos.x = (input.data[0]*256 + input.data[1])/100;

        if(fusion_pos.x > 0)
        {
            fusion_pos.x = fusion_pos.x + x_offset;
        }

        fusion_pos.y = input.data[4]*256 + input.data[5];
        if(fusion_pos.y < 0x8000)
        {
            fusion_pos.y = fusion_pos.y / 16;
        }
        else
        {
            fusion_pos.y = (fusion_pos.y - 0x10000)/16;
        }

        fusion_pub.publish(fusion_pos);   //发布毫米波雷达和摄像头的融合位置

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
        nearest.scale.x = 0.6;
        nearest.scale.y = 0.6;
        nearest.scale.z = 1.7;
        nearest.color.r = 1;
        nearest.color.g = 0;
        nearest.color.b = 0;

        if(fusion_pos.x == 0)
        {
            nearest.color.a = 0;
        }
        else
        {
            nearest.color.a = 0.8;
        }
        
        nearest.lifetime = ros::Duration();

        obj_pub.publish(nearest);    //发布最近目标几何形状
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
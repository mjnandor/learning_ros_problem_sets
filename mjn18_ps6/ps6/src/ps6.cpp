#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <iostream>
#include <string.h>


using namespace std;

geometry_msgs::Point g_box_position;
bool g_is_box ;


void cameraCallback(const osrf_gear::LogicalCameraImage& msg){
    if(msg.models.size() > 0){
       string temp_string(msg.models[0].type);
       if(temp_string.compare("shipping_box") == 0){
           g_is_box = true;
           g_box_position = msg.models[0].pose.position;
       }else{
           g_is_box == false;
       }
    }
}





int main(int argc, char **argv){
    g_box_position.x = 0;
    g_box_position.y = 0;
    g_box_position.z = 0;
    g_is_box = false;

    ROS_INFO("Problem Set 6");

    ros::init(argc, argv, "ps6_node");
    ros::NodeHandle nh;

    // initiate competition start client
    ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    while(!start_client.exists()){
        ROS_WARN("waiting for start service....");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected to start client");
    std_srvs::Trigger start_srv;

    //initiate conveyor belt client
    ros::ServiceClient belt_client = nh.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    while(!belt_client.exists()){
        ROS_WARN("waiting for belt client....");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected to belt client");
    osrf_gear::ConveyorBeltControl belt_srv;

    //initiate drone client
    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    while(!drone_client.exists()){
        ROS_WARN("waiting for drone client....");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected to drone client");
    osrf_gear::DroneControl drone_srv;
    
    //subscribe to logical camera 2
    ros::Subscriber camera_subscriber = nh.subscribe("/ariac/logical_camera_2", 1, cameraCallback);

    

    //start competition
    if(start_client.call(start_srv)){
        ROS_INFO("Successfully started competition");
    }

    //get belt moving
    belt_srv.request.power = 100;
    belt_srv.response.success = false;
    while (belt_srv.response.success == false){
        belt_client.call(belt_srv);
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("started belt");
    
    bool stop_the_box = false;
    bool is_centered = false;


    //start checking to see if box is centered
    while(stop_the_box == false){
        ros::spinOnce();
        if(g_is_box == true){
            if((g_box_position.z) > 0.0){
            stop_the_box = true;
            }
        }
    }

    //escape the while loop once a box is centered, stop the belt    
    belt_srv.request.power = 0;
    if(belt_client.call(belt_srv)){
        ROS_INFO("stopped belt");
        ROS_INFO("final belt position: %f", g_box_position.z);
    }
    
    //pause for requested time
    ros::Duration(5.0).sleep(); 

    //restart conveyor
    belt_srv.request.power = 100;
    if(belt_client.call(belt_srv)){
        ROS_INFO("started belt");
    }

    //wait for drone to pick up shipment
    drone_srv.request.shipment_type = "order_0_shipment_0";
    drone_client.call(drone_srv);
    while(drone_srv.response.success == false){
        drone_client.call(drone_srv);
    }

    //stop conveyor
    belt_srv.request.power = 0;
    if(belt_client.call(belt_srv)){
        ROS_INFO("stopped belt");
        ROS_INFO("INCOMING DRONE");
    }
}




#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <minimal_nodes_service/commandParameters.h>





double amplitude1 = 0.0;
double freq = 0.0;








int main(int argc, char **argv){

    ros::init(argc, argv, "parameter_setter");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<minimal_nodes_service::commandParameters>("wave_parameters");
    minimal_nodes_service::commandParameters srv;

    srv.request.amplitude = 0.0;
    srv.request.frequency = 0.0;

    while(ros::ok())
    {
        ROS_INFO("Enter an amplitude");
        std::cin>>srv.request.amplitude;

        ROS_INFO("Enter a frequency (Hz)");
        std::cin>>srv.request.frequency;

        //srv.request.amplitude = amplitude1;
        //srv.request.frequency = freq;
        if(client.call(srv))
        {
            ROS_INFO("here");
        }

    }




    return 0;






}

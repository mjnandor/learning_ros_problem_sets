#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <minimal_nodes_service/commandParameters.h>


//std_msgs::Float64 amplitude = 0.0; //m/s
//std_msgs::Float64 freq = 0; //Hz

double amplitude = 0.0;
double freq = 0.0;

bool callback(minimal_nodes_service::commandParametersRequest& request, minimal_nodes_service::commandParametersResponse& response)
{
    amplitude = request.amplitude;
    freq = request.frequency;
    //ROS_WARN(amplitude);

    response.confirm = true;

    return true;
}




int main(int argc, char **argv) {


    float time = 0;


    ros::init(argc, argv, "sin_commander");
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("velocityCommand", 1);

    ros::ServiceServer service = n.advertiseService("wave_parameters", callback);

    double run_rate = 1000; // Hz
    double dt = 1/run_rate;//sec
    ros::Rate naptime(run_rate);



    std_msgs::Float64 vel_cmd; //create a variable of type "Float64",
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format,
    // so subscribers know how to interpret the serialized data transmission

    vel_cmd.data = 0.0;




    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok())
    {
        time += dt;
        vel_cmd.data = amplitude*sin(2*3.14*time*freq);
        my_publisher_object.publish(vel_cmd);
        ros::spinOnce();
        naptime.sleep();

    }



    return 0;
}

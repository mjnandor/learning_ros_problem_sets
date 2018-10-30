#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); // name of this node will be "minimal_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("velocityCommand", 1);
    float run_rate = 1000; // Hz
    float dt = 1/run_rate;//sec
    ros::Rate naptime(run_rate);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups

    std_msgs::Float64 vel_cmd; //create a variable of type "Float64",
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format,
    // so subscribers know how to interpret the serialized data transmission

    vel_cmd.data = 0.0;
    float amplitude = 0.0; //m/s
    float freq = 0; //Hz
    float time = 0;




    bool first_time = true;

    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok())
    {
        if (first_time == true)
        {
            ROS_INFO("Enter an amplitude");
            std::cin>>amplitude;

            ROS_INFO("Enter a frequency (Hz)");
            std::cin>>freq;
            first_time = false;
        }
        time += dt;
        vel_cmd.data = amplitude*sin(2*3.14*time*freq);
        my_publisher_object.publish(vel_cmd);
        naptime.sleep();

    }
}

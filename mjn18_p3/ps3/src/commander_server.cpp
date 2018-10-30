#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <ps3/commander_srvAction.h>



using namespace std;


double vel_command;
ros::Publisher my_publisher_object;
std_msgs::Float64 vel_com;

class commanderActionServer{
private:

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<ps3::commander_srvAction> as_;

    ps3::commander_srvGoal inputs_;
    ps3::commander_srvResult result_;
    ps3::commander_srvFeedback progress_;


public:
    commanderActionServer();

    ~commanderActionServer(void){}

    void executeCB(const actionlib::SimpleActionServer<ps3::commander_srvAction>::GoalConstPtr& goal);
};





commanderActionServer::commanderActionServer() :
    as_(nh_, "wave_input", boost::bind(&commanderActionServer::executeCB, this, _1), false){

    as_.start();
}


void commanderActionServer::executeCB(const actionlib::SimpleActionServer<ps3::commander_srvAction>::GoalConstPtr& inputs)
{
    //
    int run_rate = 100; // Hz
    float dt = 0.01;
    float run_time = 0;
    ros::Rate timer(run_rate);

    //calculate number of cycles to run
    int running_cycles = run_rate*(inputs->numCycles)/(inputs->frequency);

    while(running_cycles > 0)
    {
        vel_command = (inputs->amplitude)*sin(2*3.14*(inputs->frequency)*run_time);
        vel_com.data = vel_command;
        my_publisher_object.publish(vel_com);
        running_cycles--;
        run_time += dt;
        timer.sleep();
    }


    vel_command = 0;
    vel_com.data = 0;
    my_publisher_object.publish(vel_com);
    result_.did_it_happen = true;
    as_.setSucceeded(result_);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "commander_server_node");
    ros::NodeHandle n;
    vel_command = 0;
    vel_com.data = 0;


    commanderActionServer as_object;

    my_publisher_object = n.advertise<std_msgs::Float64>("velocity_command", 1);
    my_publisher_object.publish(vel_com);

    ros::spin();

    return 0;
}

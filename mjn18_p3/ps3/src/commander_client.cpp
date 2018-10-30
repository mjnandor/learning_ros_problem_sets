#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>

#include<ps3/commander_srvAction.h>


using namespace std;

ros::Time begin_time;
ros::Time end_time;
double elapsed_time;

void doneCb(const actionlib::SimpleClientGoalState& state, const ps3::commander_srvResultConstPtr& result){
    ROS_INFO("done");
    end_time = ros::Time::now();
    elapsed_time = (end_time.toSec() - begin_time.toSec());
    ROS_INFO("elapsed time = %f", elapsed_time);

}







int main(int argc, char**argv)
{
    ros::init(argc, argv, "ps3_commander_client");

    ps3::commander_srvGoal wave_inputs;//create wave inputs

    actionlib::SimpleActionClient<ps3::commander_srvAction> action_client("wave_input", true);
    //creating client

    ROS_INFO("waiting for server");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));

    if(!server_exists){
        ROS_WARN("could not connect");
        return 0;
    }


    ROS_INFO("connected to action server");


    while(true)
    {
        cout << "enter frequency (Hz)  ";
        cin >> wave_inputs.frequency;
        cout << endl;
        cout << "enter amplitude  ";
        cin >> wave_inputs.amplitude;
        cout << endl;
        cout << "enter number of cycles  ";
        cin >> wave_inputs.numCycles;

        begin_time = ros::Time::now();
        action_client.sendGoal(wave_inputs, &doneCb);



    }
}

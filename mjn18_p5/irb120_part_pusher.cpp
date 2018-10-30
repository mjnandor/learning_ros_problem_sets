//Mark Nandor problem set 5 (mjn18)
//The Part Pusher

#include <ros/ros.h>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std; // avoids having to say: std::string, std::cout, etc
#include <irb120_fk_ik/irb120_kinematics.h>  //access to forward and inverse kinematics
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
// this is useful to keep the motion planner generic
#include "robot_specific_fk_ik_mappings.h" //these two files are needed to provide robot-specific info to generic planner
#include "robot_specific_names.h"

#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>

//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>


std::vector<double> g_planner_joint_weights{3, 3, 2, 1, 1, 0.5}; // weights for path planning

//stuff about the object
string g_object_name("gear_part");  // name of part
int g_found_object_code; // let's the program know magic has happened
geometry_msgs::PoseStamped g_perceived_object_pose; // received part pose
geometry_msgs::PoseStamped g_goal_pose; // where to push it to

ros::Publisher *g_pose_publisher; //this is important for something

CartTrajPlanner *pCartTrajPlanner; // the object that makes the things

Eigen::VectorXd g_q_vec_arm_Xd; // arm pose in joint states


//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    }
    else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);  //this is to enable display of pose of found object in rviz
    }
    else {
        ROS_WARN("object not found!");
    }
}




int main(int argc, char** argv){
    ros::init(argc, argv, "gear_pusher"); //name the node
    ros::NodeHandle nh; //Fine
    Eigen::Affine3d start_flange_affine, end_flange_affine, goal_flange_affine;  // start and goal tool flange affines
    std::vector<Eigen::VectorXd> optimal_path; // joint space optimal path
    trajectory_msgs::JointTrajectory new_trajectory; //stuff stuff here

    double gear_radius = 0.04;

    g_goal_pose.pose.position.x = 0.3;
    g_goal_pose.pose.position.y = 0.0;
    g_goal_pose.pose.position.z = 0.0;


    //set up the action server magic
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout = false;

    ROS_INFO("Waiting for Object Finder Server");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); //
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }

    ROS_INFO("Found the damn server");
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder

    //the following is an std::vector of affines.  It describes a path in Cartesian coords, including orientations
    //not needed yet; is constructed inside the generic planner by interpolation
    //std::vector<Eigen::Affine3d> affine_path;
    Eigen::Matrix3d R_down; //define an orientation corresponding to toolflange pointing down
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
    z_axis << 0, 0, -1; //points flange down
    x_axis << -1, 0, 0; //arbitrary
    y_axis = z_axis.cross(x_axis); //construct y-axis consistent with right-hand coordinate frame
    R_down.col(0) = x_axis;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;  //SHOULD GET FIXED: hard-coded pose can result in ugly/dangerous motion
    int nsteps = 100; //will need to specify how many interpolation points in Cartesian path; this is pretty coarse
    double arrival_time = 3; //will  need to specify arrival time for a Cartesian path


    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(NJNTS); //generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0; //assumes arm starts in this pose; better would be  to subscribe to joint_states to get actual angles


    //our irb120 control  interface uses this topic to receive trajectories
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    //somewhat odd construction: a pointer to an object of type CartTrajPlanner, with arguments provided
    //that are pointers to forward and inverse kinematic functions.  This is to keep the planner generic,
    //and defer WHICH robot FK and IK to use until run time; Uses virtual functions for this.
    pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
    //the planner needs to define penalty weights to optimize a path
    pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
    //to fill out a trajectory, need to provide the joint names; these are contained in a robot-specific header file
    pCartTrajPlanner->set_joint_names(g_jnt_names);

    optimal_path.clear(); //reset this std::vector before  each use, else  will have old values persisting
    optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
    optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose--not very useful; but can "warm up" control
    //publish/subscribe interface
    arrival_time = 1; //move should require zero time, but provide something small

    //function call from library (Class) CartTrajPlanner: converts a joint-space path to a joint-space trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory;
    ros::Duration(1).sleep();



    //xxxxxxxxxxxxxx  the following makes an inquiry for the pose of the part of interest
    //specify the part name, send it in the goal message, wait for and interpret the result
    object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server

    finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
    //check the result code to see if object was found or not
    if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object!");
    }
    else {
        ROS_WARN("object not found!  Quitting");
        return 1;
    }
    //xxxxxxxxxx   done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion


    //computing where the robot should go to push the gear - correct the x first

    //go to pushing position in x
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.x - g_goal_pose.pose.position.x >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x + 2*gear_radius, g_perceived_object_pose.pose.position.y, 0.3;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x - 2*gear_radius, g_perceived_object_pose.pose.position.y, 0.3;
    }
    goal_flange_affine.translation() = flange_origin;

    nsteps = 100;
    //compute path
    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)


    //descend down

    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.x - g_goal_pose.pose.position.x >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x + 2*gear_radius, g_perceived_object_pose.pose.position.y, 0.0001;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x - 2*gear_radius, g_perceived_object_pose.pose.position.y, 0.0001;
    }
    goal_flange_affine.translation() = flange_origin;

    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)

    //push in x

    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.x - g_goal_pose.pose.position.x >= 0)
    {
        flange_origin << g_goal_pose.pose.position.x + 1*gear_radius, g_perceived_object_pose.pose.position.y, 0.0001;
    }else{
        flange_origin << g_goal_pose.pose.position.x - 1*gear_radius, g_perceived_object_pose.pose.position.y, 0.0001;
    }
    goal_flange_affine.translation() = flange_origin;

    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)

    //raise back up
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.x - g_goal_pose.pose.position.x >= 0)
    {
        flange_origin << g_goal_pose.pose.position.x + 1*gear_radius, g_perceived_object_pose.pose.position.y, 0.3;
    }else{
        flange_origin << g_goal_pose.pose.position.x - 1*gear_radius, g_perceived_object_pose.pose.position.y, 0.3;
    }
    goal_flange_affine.translation() = flange_origin;

    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)

    //update part position
    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb);
    finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }










    //now push in y

    //move to position
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.y - g_goal_pose.pose.position.y >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y + 2*gear_radius, 0.3;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y - 2*gear_radius, 0.3;
    }
    goal_flange_affine.translation() = flange_origin;

    nsteps = 100;

    //compute path
    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)





    //descend
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.y - g_goal_pose.pose.position.y >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y + 2*gear_radius, 0.0001;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y - 2*gear_radius, 0.0001;
    }
    goal_flange_affine.translation() = flange_origin;

    nsteps = 100;
    //compute path
    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)



    //PUUUUUUUUUUUUUUSSSH
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.y - g_goal_pose.pose.position.y >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x, g_goal_pose.pose.position.y + 1*gear_radius, 0.0001;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x, g_goal_pose.pose.position.y - 1*gear_radius, 0.0001;
    }
    goal_flange_affine.translation() = flange_origin;

    nsteps = 100;
    //compute path
    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)

    //ascend
    goal_flange_affine.linear() = R_down;
    if(g_perceived_object_pose.pose.position.y - g_goal_pose.pose.position.y >= 0)
    {
        flange_origin << g_perceived_object_pose.pose.position.x, g_goal_pose.pose.position.y + 1*gear_radius, 0.3;
    }else{
        flange_origin << g_perceived_object_pose.pose.position.x, g_goal_pose.pose.position.y - 1*gear_radius, 0.3;
    }
    goal_flange_affine.translation() = flange_origin;

    nsteps = 100;
    //compute path
    g_q_vec_arm_Xd = optimal_path.back();
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }

    //convert to trajectory
    arrival_time = 2.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)
































}







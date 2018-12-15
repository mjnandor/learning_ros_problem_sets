//ps9 shipment filler - nandor

//based on shipment_filler.cpp

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector2.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>


//bin inventory does stuff
#include<bin_inventory/bin_inventory.h>

#include<xform_utils/xform_utils.h>



osrf_gear::Order g_order;
bool g_got_order = false;

geometry_msgs::Pose g_camera_pose;
geometry_msgs::Pose g_bad_part_pose_wrt_camera;
geometry_msgs::Pose g_bad_part_pose_wrt_world;


//order callback - gets order from ARIAC
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

    g_order = *msg;
    g_got_order = true;
    ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "" : "s");
    ROS_INFO_STREAM(g_order);
}



void qc1_callback(const osrf_gear::LogicalCameraImage& camera_msg){
    g_camera_pose = camera_msg.pose;
    
    if (camera_msg.models.size() > 0){
        g_bad_part_pose_wrt_camera = camera_msg.models[0].pose;
    }


}

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}





//main function

int main(int argc, char** argv){


    geometry_msgs::PoseStamped box_pose_wrt_world;
    //ROS setup
    ros::init(argc, argv, "shipment_filler_ps9");
    ros::NodeHandle nh;
    int ans;

    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    ROS_INFO("instantiating a BoxInspector");
    BoxInspector2 boxInspector(&nh);

    ROS_INFO("instantiating a binInventory object");
    BinInventory binInventory(&nh);
    inventory_msgs::Inventory current_inventory;

    ROS_INFO("Connecting to Drone");
    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl droneControl;

    // initiate competition start client
    ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    while(!start_client.exists()){
        ROS_WARN("waiting for start service....");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected to start client");
    std_srvs::Trigger start_srv;

    //start competition
    if(start_client.call(start_srv)){
        ROS_INFO("Successfully started competition");
    }

    //subscribe to quality control camera 2
    ros::Subscriber qc1_subscriber_object = nh.subscribe("/ariac/quality_control_sensor_1", 1, qc1_callback);

    XformUtils xformUtils;

    //get an order 
    ros::Subscriber order_sub = nh.subscribe("ariac/orders", 5, orderCallback);
    ROS_INFO("waiting for order...");
    while (!g_got_order) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (!g_got_order) ROS_INFO("waiting");

    }

    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
    vector<osrf_gear::Model> desired_models_wrt_world;
    vector<osrf_gear::Model> satisfied_models_wrt_world;
    vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
    vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
    vector<osrf_gear::Model> missing_models_wrt_world;
    vector<osrf_gear::Model> orphan_models_wrt_world;
    vector<int> part_indices_missing;
    vector<int> part_indices_misplaced;
    vector<int> part_indices_precisely_placed;

    int nparts;
    bool status;

    inventory_msgs::Part current_part;

    //move box to Q1
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //update box pose
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }

    //fill in desired_models_wrt_world vector, transform box coordinates to world coordinates
    int num_parts = g_order.shipments[0].products.size();
    cout << "number of parts in the order:" << num_parts << endl;
    desired_models_wrt_world.resize(num_parts);
    vector<osrf_gear::Model> desired_models_wrt_box;
    desired_models_wrt_box.resize(num_parts);
    //extract parts from order in box coordinates
    for (int i = 0; i < num_parts; i++){
        desired_models_wrt_box[i].type = g_order.shipments[0].products[i].type;
        desired_models_wrt_box[i].pose.position = g_order.shipments[0].products[i].pose.position;
        desired_models_wrt_box[i].pose.orientation = g_order.shipments[0].products[i].pose.orientation;
    }

    //convert to world coordinates
    Eigen::Affine3d box_affine_wrt_world;
    box_affine_wrt_world = xformUtils.transformPoseToEigenAffine3d(box_pose_wrt_world);
    for (int i = 0; i < num_parts; i++){
        Eigen::Affine3d part_wrt_box, part_wrt_world;
        part_wrt_box = xformUtils.transformPoseToEigenAffine3d(desired_models_wrt_box[i].pose);
        part_wrt_world = box_affine_wrt_world*part_wrt_box;
        desired_models_wrt_world[i].type = desired_models_wrt_box[i].type;
        desired_models_wrt_world[i].pose = xformUtils.transformEigenAffine3dToPose(part_wrt_world);

    }

    //run box inspector at Q1
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }

    //check with quality camera
    if (boxInspector.get_bad_part_Q1(current_part)) {
        ROS_INFO("found bad part: ");
        //ROS_INFO_STREAM(current_part<<endl);


        //run callback, get update from quality camera
        ros::spinOnce();

        //translate from camera frame to world frame
        Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;

        cam_wrt_world = xformUtils.transformPoseToEigenAffine3d(g_camera_pose);
        part_wrt_cam = xformUtils.transformPoseToEigenAffine3d(g_bad_part_pose_wrt_camera);
        part_wrt_world = cam_wrt_world*part_wrt_cam;
        g_bad_part_pose_wrt_world = xformUtils.transformEigenAffine3dToPose(part_wrt_world);
        
        //compare bad part pose to pose of all parts in box
        num_parts = orphan_models_wrt_world.size();
        double part_distance_from_bad[num_parts];
       
        for (int i = 0; i < num_parts; i++){
            double x_dist = g_bad_part_pose_wrt_world.position.x - orphan_models_wrt_world[i].pose.position.x;
            double y_dist = g_bad_part_pose_wrt_world.position.y - orphan_models_wrt_world[i].pose.position.y;
            
            double temp_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    
            part_distance_from_bad[i] = temp_dist;
            //ROS_INFO("%f, %f",(float)i, part_distance_from_bad[i]);
        }
        
        //find minimum distance index
        int min_index = 0;
    
        for (int i = 1; i < num_parts; i++){
            //ROS_INFO("made it here");
            if (part_distance_from_bad[i] < part_distance_from_bad[min_index]){
                min_index = i;
            }
        }
        //ROS_INFO("part index: %f", (float)min_index);
        //cout<< "minimum part index:" << min_index;
        
        //removing bad part from box
        model_to_part(orphan_models_wrt_world[min_index], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);

        //go find another piston
    
        int n_missing_part = part_indices_missing[0];

        //model_to_part(desired_models_wrt_world[n_missing_part], current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
        std::string part_name(desired_models_wrt_world[n_missing_part].type);

        ROS_INFO_STREAM("looking for part " << part_name << endl);
        int partnum_in_inventory;
        bool part_in_inventory = true;
        inventory_msgs::Part pick_part, place_part;


        binInventory.update();
        binInventory.get_inventory(current_inventory);
        part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
        if (!part_in_inventory) {
            ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
            return false; //nothing more can be done     
        }
        ROS_INFO_STREAM("found part: " << pick_part << endl);
        //specify place part:
        model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_2);

        status = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
        if (!status) {
            ROS_WARN("could not compute key pickup and place poses for this part source and destination");
        }



        ROS_INFO("attempting pick...");
    


        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("pick failed");
            status = false;
            return false;
            //gripperInterface_.release();     
        }

        ROS_INFO("moving to approach pose");
        if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
            ROS_WARN("could not move to approach pose");
            status = false;
            robotBehaviorInterface.discard_grasped_part(place_part);
            //return false;  // REMOVE THIS IF NEEDED
        }
        //place  part:
        ROS_INFO("attempting to place part");
        if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
            ROS_INFO("placement failed");
            status = false;
            return false;
        }

        //release and retract
        ROS_INFO("release and retract part");
        if (!robotBehaviorInterface.release_and_retract()) {
            ROS_INFO("placement failed");
            status = false;
            return false;
        }

    } 



    //move to quality station 2
    nprint = 0;
    conveyorInterface.move_box_Q1_to_Q2(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    cout << "enter 1 to continue: ";
    cin>>ans;



    //update box location
    status = boxInspector.get_box_pose_wrt_world2(box_pose_wrt_world);
    //ROS_INFO_STREAM(box_pose_wrt_world); 

    //update desired part positions

    //convert to world coordinates
    num_parts = g_order.shipments[0].products.size();
    box_affine_wrt_world = xformUtils.transformPoseToEigenAffine3d(box_pose_wrt_world);
    for (int i = 0; i < num_parts; i++){
        Eigen::Affine3d part_wrt_box, part_wrt_world;
        part_wrt_box = xformUtils.transformPoseToEigenAffine3d(desired_models_wrt_box[i].pose);
        part_wrt_world = box_affine_wrt_world*part_wrt_box;
        desired_models_wrt_world[i].type = desired_models_wrt_box[i].type;
        desired_models_wrt_world[i].pose = xformUtils.transformEigenAffine3dToPose(part_wrt_world);
        ROS_INFO_STREAM(desired_models_wrt_world[i].type);
        ROS_INFO_STREAM(desired_models_wrt_world[i].pose);
    }

    //call update inspection again, with Q2
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed, (int)2);
    ROS_INFO("orphaned parts in box: ");
    nparts = misplaced_models_actual_coords_wrt_world.size();
    ROS_INFO("num misplaced parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("misplaced  parts: "<<misplaced_models_actual_coords_wrt_world[i]<<endl);
    }

    cout << "enter 1 to continue: ";
    cin>>ans;

    //move misplaced parts
    
    for (int i = 0; i < nparts; i++){
        //move to misplaced part
        Part source_part, destination_part;
        model_to_part(misplaced_models_actual_coords_wrt_world[i], source_part, inventory_msgs::Part::QUALITY_SENSOR_2);
        model_to_part(misplaced_models_desired_coords_wrt_world[i], destination_part, inventory_msgs::Part::QUALITY_SENSOR_2);
        
        ROS_INFO("Correcting Part");
        status = robotBehaviorInterface.pick_part_from_box(source_part);
        status = robotBehaviorInterface.adjust_part_location_no_release(source_part, destination_part);
        status = robotBehaviorInterface.release_and_retract();
    }   

    //move to drone
    nprint = 0;
    conveyorInterface.move_box_Q2_to_drone_depot(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        nprint++;
        if (nprint % 100 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Drone...");
        }
    }

    ros::Duration(5.0).sleep();
    //call the drone
    droneControl.request.shipment_type = g_order.shipments[0].shipment_type;
    drone_client.call(droneControl);
    


    ROS_INFO("END OF PROGRAM");
  




    

}
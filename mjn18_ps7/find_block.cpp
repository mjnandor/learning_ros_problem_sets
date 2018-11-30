//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>



static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel

        double camera_rotation = 0.1955;
        Eigen::MatrixXd mean_values;

        Eigen::MatrixXd red_pixels; //array to contain the i,j coordinates.  will be used to compute orientation
        Eigen::MatrixXd pixels_world;//array to contain pixels translated into world frame
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;

                    //place found pixels in matrix for later orientation finding
                    red_pixels.conservativeResize(npix + 1, 2);
                    pixels_world.conservativeResize(npix + 1, 2);
                    red_pixels(npix, 0) = (double)i;
                    red_pixels(npix, 1) = (double)j;
                    
                    
                    //scale to world
                    pixels_world(npix,0) = 0.002966*(((double)i)*cos(camera_rotation) - ((double)j)*sin(camera_rotation)) - 0.24477;
                    pixels_world(npix,1) = -0.003047*(((double)i)*sin(camera_rotation) + ((double)j)*cos(camera_rotation)) + 1.2217;

                    //ROS_INFO("%f, %f, %f", (float)npix, pixels_world(npix,0), pixels_world(npix,1));
                    

                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        double x_center2, y_center2;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);

            mean_values = pixels_world.colwise().mean();
            
        
            
            //ROS_INFO("%f, %f, %f, %f",x_centroid, y_centroid, mean_values(0,0), mean_values(0,1));
            

            //ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());

        //transform from camera coordinates to robot coordinates
        //float camera_rotation = 0.1955; //camera angle, radians

        block_pose_.pose.position.x = mean_values(0,0); //not true, but legal
        block_pose_.pose.position.y = mean_values(0,1); //not true, but legal
       
        
       

        //find block orientation 
        Eigen::MatrixXd pixels_centered(pixels_world.rows(), pixels_world.cols());
        
        //subtract off means
        for(int i = 0; i < pixels_world.rows(); i++){
            pixels_centered.row(i) = pixels_world.row(i) - mean_values;
        }

        //Compute Covariance
        Eigen::MatrixXd covar(pixels_world.rows(), pixels_world.cols());
        covar = (pixels_centered).transpose()*pixels_centered;

        Eigen::EigenSolver<Eigen::Matrix2d> eigs;
        eigs.compute(covar, true);
        Eigen::Vector2d e_values;
        e_values = eigs.eigenvalues().real();
        //ROS_INFO("%f, %f", (float)(eigs.eigenvectors().rows()), (float)(eigs.eigenvectors().cols()));
        int high_eigen = 0;
        if (e_values(1) > e_values(0)){
            high_eigen = 1;
        }
        Eigen::Vector2d o_vector;
        o_vector = eigs.eigenvectors().col(high_eigen).real();
        

        
        
        
        
        
        double theta = atan2(o_vector(1), o_vector(0)) + 1.57079632679;
        ROS_INFO("orientation: %f", theta);
        //ROS_INFO("%f,", theta);
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(theta); //not true, but legal
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

///Global variables
static const char WINDOW_NAME_BW[] = "window_image";

int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

char* trackbar_type="Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value="Value";

cv::Mat image;
cv::Mat image_hsv;
cv::Mat image_bw;

///The workhorse class
class FindSummit
{
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  
public:
  FindSummit();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  void imageShow (const ros::TimerEvent& e);
  
  static void Threshold_Demo( int, void* ); 
};

///Contructor
 FindSummit::FindSummit()
{
    ROS_INFO("Init Class");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
    std::string image_topic = "/quadrotor/downward_cam/camera/image";
    cv::namedWindow(WINDOW_NAME_BW, CV_WINDOW_AUTOSIZE);
    ROS_INFO("Window opened"); 
    
    cv::createTrackbar( trackbar_type, WINDOW_NAME_BW, &threshold_type, max_type, &FindSummit::Threshold_Demo, this);
    cv::createTrackbar( trackbar_value, WINDOW_NAME_BW, &threshold_value, max_value, &FindSummit::Threshold_Demo, this);
    ROS_INFO("Trackbars added"); 
    
    
    sub_ = it_.subscribeCamera(image_topic, 1, &FindSummit::imageCb, this);
    pub_ = it_.advertise("image_out", 1);   
    //cv::destroyWindow(WINDOW_NAME_BW);
	ROS_INFO("Setup completed");
  }
  
///Function image 
  void FindSummit::imageCb(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv_bridge::CvImagePtr input_bridge;
    ROS_INFO("Img recieved");
    
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8); //Maybe BGR8
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    FindSummit::Threshold_Demo(0,0);
	//cv::cvtColor(image, image_hsv , CV_RGB2HSV);
	//cv::inRange(image_hsv, cv::Scalar(170,160,60), cv::Scalar(180,256,256),image_bw);			
	//pub_.publish(input_bridge->toImageMsg());
	
	//ROS_INFO("Img show");
	//cv::imshow(WINDOW_NAME, image_bw);
	//cv::waitKey(3);
	//ROS_INFO("Img show end");
  }
  
  
///threshold trackbar function 
  void FindSummit::Threshold_Demo( int, void* )
{
   /* 0: Binary
      1: Binary Inverted
      2: Threshold Truncated
      3: Threshold to Zero
      4: Threshold to Zero Inverted
   */
   
   cv::cvtColor(image, image_hsv , CV_RGB2GRAY); //if you put hsv instead, threshold may appear with colors such as red and yellow
   cv::threshold(image_hsv, image_bw, threshold_value, max_BINARY_value, threshold_type );
   ROS_INFO("Img show");
   cv::imshow(WINDOW_NAME_BW,image_bw);
   cv::waitKey(3);
   ROS_INFO("Img show end");
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_summit");
  ROS_INFO("Ros node started");
  FindSummit find_red_summit;
  ROS_INFO("Spinng");
  ros::spin();
}

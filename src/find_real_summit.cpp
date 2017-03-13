#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

//---------------------------------------
//--------------------------------------
#include "/home/baira/catkin_ws/devel/ardrone_autonomy/msg_gen/cpp/include/ardrone_autonomy/Navdata.h"


#define DEFAULT_MIN_NUM_SIDES 4
///Global variables --------------------------------------
static const char WINDOW_NAME_BW[] = "window_image_threshold";
static const char WINDOW_NAME_FINAL[] = "window_image_contours";

int threshold_value = 33;
int threshold_type = 1;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

char* trackbar_type="Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value="Value";

cv::Mat image;
cv::Mat image_hsv;
cv::Mat image_bw;
cv::Mat image_eroded;
cv::Mat image_2;

///The workhorse class ------------------------------------
class FindSummit
{
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  ros::Subscriber camera_info_;
  ros::Publisher summit_position_;
  ros::Subscriber altitude_;
  ros::Subscriber cam_info_;
  ros::Subscriber imu_info_;

  double fx;
  double fy;
  double cx;
  double cy;
  double T;
  double altitude;
  double angle_x;
  double angular_velocity_x;
  double angle_y;
  double angular_velocity_y;
  int min_num_sides;
  int color_R;
  int color_G;
  int color_B;

public:
  FindSummit();
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void height(const ardrone_autonomy::Navdata& parrot_height);
  void cam_info(const sensor_msgs::CameraInfo& cam_parameters);
  void angle(const sensor_msgs::ImuConstPtr& hector_imu);
  void Threshold_Demo( int, void* );

};

///Contructor ---------------------------------------------
 FindSummit::FindSummit()
 {
    ROS_INFO("Init Class");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
    std::string image_topic;
    std::string summit_position_topic;
    std::string altimeter_topic;
    std::string camera_info_topic;
    std::string imu_topic;

    //int threshold_value;

    nh_.param("image_topic", image_topic, std::string("/quadrotor/downward_cam/camera/image"));
    nh_.param("cam_info_topic", camera_info_topic, std::string("/quadrotor/downward_cam/camera/camera_info"));
    nh_.param("summit_position", summit_position_topic, std::string("Estimated_position"));
    nh_.param("altitude", altimeter_topic, std::string("/quadrotor/sonar_height"));
    nh_.param("imu_topic", imu_topic, std::string("/quadrotor/raw_imu"));
    nh_.param("min_num_sides", min_num_sides, DEFAULT_MIN_NUM_SIDES);
    nh_.param("color_R", color_R, 255);
    nh_.param("color_G", color_G, 0);
    nh_.param("color_B", color_B, 0);
    //nh_.param("threshold_value", threshold_value, 33);


    cv::namedWindow(WINDOW_NAME_BW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_NAME_FINAL, CV_WINDOW_AUTOSIZE);
    ROS_INFO("Window opened");
    //revisar
    //cv::createTrackbar( trackbar_type, WINDOW_NAME_BW, &threshold_type, max_type, &FindSummit::Threshold_Demo<FindSummit>, this);
    //cv::createTrackbar( trackbar_value, WINDOW_NAME_BW, &threshold_value, max_value, &FindSummit::Threshold_Demo<FindSummit>, this);
    //ROS_INFO("Trackbars added");

    sub_ = it_.subscribeCamera(image_topic, 1, &FindSummit::imageCb, this);
    altitude_ = nh_.subscribe(altimeter_topic, 1, &FindSummit::height, this);
    imu_info_ = nh_.subscribe(imu_topic, 1, &FindSummit::angle, this);
    //pub_ = it_.advertise("image_out", 1);
    summit_position_ = nh_.advertise<geometry_msgs::PoseStamped>(summit_position_topic, 1);
    cam_info_ = nh_.subscribe(camera_info_topic, 1 , &FindSummit::cam_info, this);

    /*joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ARDroneJoy::joyCallback, this);*/
	ROS_INFO("Setup completed");
  }

 void FindSummit::height(const ardrone_autonomy::Navdata& parrot_height ){ /*extraction of the altitude*/

	 altitude= parrot_height.altd;
	// ROS_INFO("Altitude : %f", altitude);
 }

 void FindSummit::angle(const sensor_msgs::ImuConstPtr& hector_imu){

	 angle_x = hector_imu->orientation.x;
	 angular_velocity_x= hector_imu->angular_velocity.x;
	 angle_y = hector_imu->orientation.y;
	 angular_velocity_y= hector_imu->angular_velocity.y;

 }

 void FindSummit::cam_info(const sensor_msgs::CameraInfo& cam_parameters){
	//ROS_INFO("Parameters from the pinhole model");
	fx=cam_parameters.K.at(0);
    fy=cam_parameters.K.at(4);
    cx=cam_parameters.K.at(2);
    cy=cam_parameters.K.at(5);
    T=cam_parameters.K.at(1);
	//ROS_INFO("Los parametros son  (fx,fy)=(%f , %f)", fx, fy);
 }

///Function image -------------------------------------------
  void FindSummit::imageCb(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv_bridge::CvImagePtr input_bridge;
    //ROS_INFO("Img recieved!!!");

		try {
		  input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8); //Maybe BGR8
		  image = input_bridge->image;
		  image_2 = input_bridge->image;
		}
		catch (cv_bridge::Exception& ex){
		  ROS_ERROR("[draw_frames] Failed to convert image %s", ex.what());
		}

		Threshold_Demo(0,0);
	//pub_.publish(input_bridge->toImageMsg());
  }


///threshold and contour detection function -----------------------------
  void FindSummit::Threshold_Demo( int, void* )
  //void static Threshold_Demo( int, void*)
{
   std::vector< std::vector<cv::Point> > contours;
   std::vector<cv::Vec4i> hierarchy;
   std::vector<cv::Point> approx;

   cv::cvtColor(image, image_hsv , CV_RGB2GRAY);
   cv::threshold(image_hsv, image_bw, threshold_value, max_BINARY_value, threshold_type );
   //ROS_INFO("Threshold demo");

   ///---------------lets try an erode
   cv::erode(image_bw, image_eroded, 1);

   cv::imshow(WINDOW_NAME_BW,image_eroded);
   cv::waitKey(3);

   //copy
   cv::Mat image_1=image_eroded;

   //finding all contours in the image
   cv::findContours(image_1, contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
   //ROS_INFO("Find contours");
   //this function modifies the image


   //---------------------------------------------------------

   for (int i = 0; i < contours.size(); i++){
    	//ROS_INFO("Loop");
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)* 0.02, true);
		//ROS_INFO("approxPoly");

		//Skip small or non-convex objects
        if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
			continue;


		//if there are more than min_num_sides and...
		 if (approx.size() >= min_num_sides && fabs(angle_x) <= 0.5 && fabs(angular_velocity_x) <= 0.5 //angles limit
				              && fabs(angular_velocity_y) <= 0.5 && fabs(angle_y) < 0.5
				              && fabs(contourArea(cv::Mat(approx))) >= 0.3*((0.3*fx*fy)/(altitude*altitude)) //area transformation
				              && fabs(contourArea(cv::Mat(approx))) <= 2*((0.3*fx*fy)/(altitude*altitude))
				              && fabs(angular_velocity_x)*fabs(angular_velocity_y) <= 0.25 //variation limit
				              && cv::isContourConvex(cv::Mat(approx))){
			ROS_INFO("hexagon found");
			ROS_INFO("El area del objetio es: %f", fabs(contourArea(cv::Mat(approx))));
			ROS_INFO("El area supuesta es: %f", (0.3*fx*fy)/(altitude*altitude));
			//iterating through each point
			cv::Point pt;
			for(int j=0;j<approx.size();j++){
				//cv::Point point = approx[i];
				//ROS_INFO("extracting points X %d Y %d",point.x, point.y);

				if (j == approx.size()-1)
				{
					cv::line(image_2, approx.at(j), approx.at(0), cv::Scalar(0,255,0), 4);
				}
				else
				{
					cv::line(image_2, approx.at(j), approx.at(j+1), cv::Scalar(0,255,0), 4);
				}
			}

			// Centroid extraction----------------------------------------------------------------
			/// 1º Get the moments
		    cv::Moments m;
			m = cv::moments( approx, false );

			/// 2º Get the mass centers: (centroid)
		    cv::Point2f mc;
			mc = cv::Point2f( m.m10/m.m00 , m.m01/m.m00 );
			cv::line(image_2, mc, mc, cv::Scalar(0,255,255), 8); // we draw a yellow point for that mc

			/// Pinhole inverse transformation ==> (x,y,z)= f(u,v) without distortion, (u,v)=mc ---------------------
			ROS_INFO("Centroid location");
			geometry_msgs::PoseStamped centroid;
            centroid.header.stamp= ros::Time::now();
            centroid.header.seq = ros::Time::now().toSec();
		    centroid.header.frame_id = "/image_plane";
            centroid.pose.orientation.w=1;
            centroid.pose.orientation.x=0;
            centroid.pose.orientation.y=0;
            centroid.pose.orientation.z=0;
            centroid.pose.position.x=-((mc.x - cx)*altitude - (T*(mc.y - cy)*altitude)/fy)/fx;
		    centroid.pose.position.y=-((mc.y - cy)*altitude)/fy;
			centroid.pose.position.z=altitude;
			ROS_INFO("Centro de masas en pixels: (%f , %f)", mc.x, mc.y);
			ROS_INFO("La posicion del objetivo es: (%f , %f, %f)", centroid.pose.position.x, centroid.pose.position.y, centroid.pose.position.z);

		    summit_position_.publish(centroid);

		//	return centroid;
		 }
	}

    cv::imshow(WINDOW_NAME_FINAL,image_2);
    cv::waitKey(3);
}

///------------ DISTORTION (Radial + tangential) ------------------------------///
/*
  uu= ud + (1+ k1*r² + k2*r⁴ + k3*r⁶) + (2*p1*vd + p2*(r² + 2*ud²));
  vu = vd + (1+ k1*r² + k2*r⁴ + k3*r⁶) + (p1*(r²+2*vd²)+ 2*p2*ud);

  r=sqrt((ud - cx)²+ (vd - cy)²);
*/



///main ----------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_summit_shape");
  ROS_INFO("Ros node started");
  FindSummit find_shape;
  ROS_INFO("Spinng");
  ros::spin();
}

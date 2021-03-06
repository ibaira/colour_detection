#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>

#define DEFAULT_MIN_NUM_SIDES 4
#define DEFAULT_MAX_NUM_SIDES 12
#define DEFAULT_SIZE 0.21

///Global variables --------------------------------------
static const char WINDOW_NAME_BW[] = "window_image_threshold";
static const char WINDOW_NAME_FINAL[] = "window_image_final";

static const char WINDOW_NAME_INITIAL[] = "window_image_initial";
static const char WINDOW_NAME_BINARY[] = "window_image_binary";
static const char WINDOW_NAME_ERODE1[] = "window_image_erode1";
static const char WINDOW_NAME_BLUR[] = "window_image_blur";
static const char WINDOW_NAME_DILATE1[] = "window_image_dilated1";

int threshold_value = 33;
int threshold_type = 1;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

int h_min = 118;
int h_max = 121;
int s_min = 241;
int s_max = 256;
int v_min = 253;
int v_max = 256;

int images_read = 0;
int times_detected = 0;
int false_positives = 0;
int obj_detected = 0;

//char* trackbar_type="Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_hue="Hue";
char* trackbar_saturation="Saturation";
char* trackbar_value="Value";

cv::Mat image;
cv::Mat image_hsv;
cv::Mat image_bw;
cv::Mat image_eroded;
cv::Mat image_2;


///The workhorse class ------------------------------------
class FindSummit{
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  ros::Subscriber camera_info_;
  ros::Publisher summit_position_;
  ros::Subscriber altitude_;
  ros::Subscriber cam_info_;
  ros::Subscriber imu_info_;
  //ros::Publisher reset_predict;

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
  double linear_acceleration_x;
  double linear_acceleration_y;
  int min_num_sides;
  int max_num_sides;
  double teoric_size_m;
  //ros::Timer timer_;

  //Transformations
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

public:
  FindSummit();
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void height(const sensor_msgs::RangeConstPtr& hector_height);
  void cam_info(const sensor_msgs::CameraInfo& cam_parameters);
  void angle(const sensor_msgs::ImuConstPtr& hector_imu);
  //void spin(const ros::TimerEvent& e);
  void Threshold_Demo( int, void* );
};

template <class obj> void tmp( int value, void* oooo){ //function that let us use the function: threshold_demo. Because we need threshold_demo not to be part of the class FindSummit
    obj *newObj = (obj*)oooo;
    newObj->Threshold_Demo(value, newObj); // value for the trackbar obj
}

///Contructor ---------------------------------------------
  FindSummit::FindSummit(){
    ROS_INFO("Init Class");
    ros::NodeHandle nh_("~");
	  image_transport::ImageTransport it_(nh_);
    std::string image_topic;
    std::string summit_position_topic;
    std::string altimeter_topic;
    std::string camera_info_topic;
    std::string imu_topic;


    nh_.param("image_topic", image_topic, std::string("/quadrotor/downward_cam/camera/image"));
    nh_.param("cam_info_topic", camera_info_topic, std::string("/quadrotor/downward_cam/camera/camera_info"));
    nh_.param("summit_position_topic", summit_position_topic, std::string("Estimated_position"));
    nh_.param("altitude", altimeter_topic, std::string("/quadrotor/sonar_height")); // /quadrotor/altimeter has less limitations
    nh_.param("imu_topic", imu_topic, std::string("/quadrotor/raw_imu"));
    nh_.param("min_num_sides", min_num_sides, DEFAULT_MIN_NUM_SIDES);
    nh_.param("max_num_sides", max_num_sides, DEFAULT_MAX_NUM_SIDES);
    nh_.param("h_min", h_min, 0);
    nh_.param("h_max", h_max, 256);
    nh_.param("s_min", s_min, 0);
    nh_.param("s_max", s_max, 256);
    nh_.param("v_min", v_min, 0);
    nh_.param("v_max", v_max, 256);
    nh_.param("teoric_size_m", teoric_size_m, DEFAULT_SIZE);
    //nh_.param("reset_predict", reset_predict, std::string("/reset_predict"));

    cv::namedWindow(WINDOW_NAME_BW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_NAME_FINAL, CV_WINDOW_AUTOSIZE);
    ROS_INFO("Window opened");

    //cv::createTrackbar( trackbar_type, WINDOW_NAME_BW, &threshold_type, max_type, &tmp<FindSummit>, this);
    cv::createTrackbar( "h_min", WINDOW_NAME_BW, &h_min, 256, &tmp<FindSummit>, this);
    cv::createTrackbar( "h_max", WINDOW_NAME_BW, &h_max, 256, &tmp<FindSummit>, this);
    cv::createTrackbar( "s_min", WINDOW_NAME_BW, &s_min, 256, &tmp<FindSummit>, this);
    cv::createTrackbar( "s_max", WINDOW_NAME_BW, &s_max, 256, &tmp<FindSummit>, this);
    cv::createTrackbar( "v_min", WINDOW_NAME_BW, &v_min, 256, &tmp<FindSummit>, this);
    cv::createTrackbar( "v_max", WINDOW_NAME_BW, &v_max, 256, &tmp<FindSummit>, this);

    //cv::createTrackbar( trackbar_type, WINDOW_NAME_BW, &threshold_type, max_type, &FindSummit::Threshold_Demo, this);
    //cv::createTrackbar( trackbar_value, WINDOW_NAME_BW, &threshold_value, max_value, &FindSummit::Threshold_Demo, this);
    //ROS_INFO("Trackbars added");

    sub_ = it_.subscribeCamera(image_topic, 1, &FindSummit::imageCb, this);
    altitude_ = nh_.subscribe(altimeter_topic, 1, &FindSummit::height, this);
    imu_info_ = nh_.subscribe(imu_topic, 1, &FindSummit::angle, this);
    //pub_ = it_.advertise("image_out", 1);
    summit_position_ = nh_.advertise<geometry_msgs::PoseStamped>(summit_position_topic, 1);
    cam_info_ = nh_.subscribe(camera_info_topic, 1 , &FindSummit::cam_info, this);

    /*joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ARDroneJoy::joyCallback, this);*/

    /*timer_ = nh_.createTimer(ros::Duration(0.09), &FindSummit::spin, this); //0.09 con 0.04 es el limite para que vaya bien y no sume falsos negativos a lo loco
    timer_.stop();*/
	ROS_INFO("Setup completed");
  }

 void FindSummit::height(const sensor_msgs::RangeConstPtr& hector_height ){

	 altitude= hector_height->range;

     //Filtro paso banda
     if(altitude > 6.5 || altitude < 5.5){
         altitude = 6.0;
     }
    //ROS_INFO("Altitude : %f", altitude);
 }

 /*void FindSummit::spin(const ros::TimerEvent& e){
     Threshold_Demo(0,0);//llamo al tratamiento con una frecuencia de 5Hz
     // ROS_INFO("SPIN");
 }*/

 void FindSummit::angle(const sensor_msgs::ImuConstPtr& hector_imu){

	 angle_x = hector_imu->orientation.x;
	 angular_velocity_x= hector_imu->angular_velocity.x;
	 angle_y = hector_imu->orientation.y;
	 angular_velocity_y= hector_imu->angular_velocity.y;
     linear_acceleration_x = hector_imu->linear_acceleration.x;
     linear_acceleration_y = hector_imu->linear_acceleration.y;

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
          images_read++;  //////// Count images we read
        }
		catch (cv_bridge::Exception& ex){
		  ROS_ERROR("[draw_frames] Failed to convert image %s", ex.what());
		}
    //if(images_read >= 1) timer_.start();

        Threshold_Demo(0,0);
	//pub_.publish(input_bridge->toImageMsg());
  }


///threshold and contour detection function -----------------------------
  void FindSummit::Threshold_Demo( int value, void* abd)
{
   std::vector< std::vector<cv::Point> > contours;
   std::vector<cv::Vec4i> hierarchy;
   std::vector<cv::Point> approx;
   double tarjet_size;
   //cv::Mat bad =

   //cv::imshow(WINDOW_NAME_INITIAL,image);
   //cv::waitKey(3);

   cv::cvtColor(image, image_hsv , CV_BGR2HSV);
   //cv::threshold(image_hsv, image_bw, threshold_value, max_BINARY_value, threshold_type );
   //ROS_INFO("Threshold demo");
   cv::inRange(image_hsv,cv::Scalar(h_min,s_min,v_min),cv::Scalar(h_max,s_max,v_max),image_bw);
   cv::Mat image_binary = image_bw;
   cv::imshow(WINDOW_NAME_BINARY,image_binary);
   cv::waitKey(3);

   image_eroded = image_bw;///////////////////////

   ///---------------lets try an erode
   //cv::erode(image_bw, image_eroded, 100);
   //cv::Mat image_erode1 = image_eroded;
   ///cv::imshow(WINDOW_NAME_ERODE1,image_erode1);
   ///cv::waitKey(3);

   cv::medianBlur(image_eroded, image_eroded, 3);
   cv::Mat image_blur = image_eroded;
   cv::imshow(WINDOW_NAME_BLUR,image_blur);
   cv::waitKey(3);

   //cv::bilateralFilter(image_eroded, image_eroded, 1, 2, 0.5);
   ///cv::dilate(image_eroded, image_eroded, 100);
   ///cv::Mat image_dilated = image_eroded;
   ///cv::imshow(WINDOW_NAME_DILATE1,image_dilated);
   ////cv::waitKey(3);

   ///cv::erode(image_eroded, image_eroded, 100);
   ///cv::dilate(image_eroded, image_eroded, 100);
   ///cv::erode(image_eroded, image_eroded, 100);
   ///cv::dilate(image_eroded, image_eroded, 100);
   ///cv::erode(image_eroded, image_eroded, 100);

   cv::imshow(WINDOW_NAME_BW,image_eroded); //Final blanco y negro
   cv::waitKey(3);

   //copy
   cv::Mat image_1=image_eroded;

   //finding all contours in the image
   cv::findContours(image_1, contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
   //ROS_INFO("Find contours");
   //this function modifies the image
   //ros::Rate loop_rate(0.1);
   //---------------------------------------------------------
   tarjet_size = teoric_size_m;
   obj_detected = 0;
   for (int i = 0; i < contours.size(); i++){
    	//ROS_INFO("Loop");
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)* 0.02, true);
        //ROS_INFO("approxPoly");

		//Skip small or non-convex objects
        if (std::fabs(cv::contourArea(contours[i])) < 0.5*((tarjet_size*fx*fy)/(altitude*altitude)) || !cv::isContourConvex(approx) || std::fabs(cv::contourArea(contours[i])) > 2*((tarjet_size*fx*fy)/(altitude*altitude)))
			continue;

        // we get the expectedsize from the class
        //if there are more than min_num_sides and...
        if (approx.size() >= min_num_sides && approx.size() <= max_num_sides && fabs(angle_x) <= 0.5 && fabs(angular_velocity_x) <= 0.4 //angles limit 0.5
                              && fabs(angular_velocity_y) <= 0.4 && fabs(angle_y) <= 0.5
                              && fabs(linear_acceleration_x) <= 1 && fabs(linear_acceleration_y) <= 1
                              && fabs(linear_acceleration_x*linear_acceleration_y) <= 0.7
                              //area transformation -> real~= 693mm x (626-x) ~= 2,1m² - And we allow +-30% of difference
                              && fabs(contourArea(cv::Mat(approx))) >= 0.7*((tarjet_size*fx*fy)/(altitude*altitude))
                              && fabs(contourArea(cv::Mat(approx))) <= 1.3*((tarjet_size*fx*fy)/(altitude*altitude))
                              && fabs(angular_velocity_x)*fabs(angular_velocity_y) <= 0.25 //variation angular_velocity limit
                              && cv::isContourConvex(cv::Mat(approx))){
            ROS_INFO("Summit_xl found");
            times_detected++; // it counts how many times we detect the tarjet

            if(obj_detected >= 1){
                false_positives++;
            }
            else{
                obj_detected++;
            }
            ROS_INFO("El area del objetio es: %f pixels/ %.3f m2", fabs(contourArea(cv::Mat(approx))), fabs(contourArea(cv::Mat(approx)))*((altitude*altitude)/(fx*fy)) );
            ROS_INFO("El area teorica es: %f pixels/  %.3f m2", (tarjet_size*fx*fy)/(altitude*altitude), tarjet_size); //2100cm2?
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
            ROS_INFO("Centro de masas en pixels del objetivo: (%f , %f)", mc.x, mc.y);
            ROS_INFO("La posicion del objetivo es: (%f , %f, %f) en metros respecto a la camara", centroid.pose.position.x, centroid.pose.position.y, centroid.pose.position.z);


            ROS_INFO("RESULTS");
            ROS_INFO("imag_obtenidas = %d / veces detectado = %d", images_read, times_detected);
            ROS_INFO("Falsos negativos = %d", images_read - times_detected + false_positives);
            ROS_INFO("Falsos positivos = %d \n\n", false_positives);

            summit_position_.publish(centroid);
         }

         /*else{ //boolean para reseteo de juanjo
             reset_predict.publish(1);
         }*/
	}

    cv::imshow(WINDOW_NAME_FINAL,image_2);
    cv::waitKey(3);
}

///------------ DISTORTION (Radial + tangential) ------------------------------///
/*Al revés!!!!!!!
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

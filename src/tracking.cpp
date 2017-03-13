#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//se debera hacer el control con una frecuencia de menor que 1/(1/20)* frecuencia de recepcion de posicion del coche aprox
//Estimacion de la posicion se emite a 13Hz -> 0.03s - 0.2s => 50*13 = 650Hz , 0.0006s - 0.04s

double sumatorio_error_x = 0;
double sumatorio_error_y = 0;

double derror_x = 0;
double derror_y = 0;

double position_x0;
double position_y0;

float t_ref = 0;

class ARDroneJoy
{
  public:
    ARDroneJoy();

    geometry_msgs::Twist vel;
    std_msgs::Empty emptyMsg;
    std_srvs::Empty emptySrv;

    double current_vel;
    double position_x;
    double position_y;
  
    ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
    ros::Publisher takeoff_pub ;
    ros::Publisher land_pub;
    ros::Publisher reset_pub;
    ros::Subscriber joy_sub_;//! It will be suscribed to the joystick
    ros::ServiceClient client_1;//!togglecam
    ros::ServiceClient client_2;//!flattrim
    ros::Subscriber summit_position_;
    ros::Publisher position_quad_base_;
    ros::Publisher estimated_summit_position_;

    std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity

    //Transformations
    tf::TransformListener tf_listener;
    tf::StampedTransform transform, transform_world;

    void  flattrim(float duration);
    void  land(float duration);
    void  takeoff(float duration);
    void  giro(float duration, char  sentido);
    void  desplazamiento_x(float duration, char sentido);
    void  desplazamiento_y(float duration, char sentido);
    void  tracking(const geometry_msgs::PoseStamped& centroid);

  private:
    ros::NodeHandle nh_;
 };


ARDroneJoy::ARDroneJoy()
{
	std::string summit_position_topic;
    std::string position_quad_base_topic;
    std::string estimated_summit_position_topic;
	current_vel = 1.0;

    nh_.param("cmd_topic_vel", cmd_topic_vel_, std::string("quadrotor/cmd_vel"));
    nh_.param("summit_position_topic", summit_position_topic, std::string("find_summit_final/Estimated_position"));
    nh_.param("position_quad_base_topic", position_quad_base_topic, std::string("tracking/tarjet_position"));
    nh_.param("estimated_summit_position_topic", estimated_summit_position_topic, std::string("world_estimated_position"));

    ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());

  	// Publish through the node handle Twist type messages to the ARDrone_ctrl/command topic
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	//Publish to topics
	takeoff_pub = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	land_pub = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
	reset_pub = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);
    position_quad_base_ = nh_.advertise<geometry_msgs::PoseStamped>(position_quad_base_topic, 1);
    estimated_summit_position_ = nh_.advertise<geometry_msgs::PoseStamped>(estimated_summit_position_topic, 1);

	//Ask for services
	client_1 = nh_.serviceClient<std_srvs::Empty>("ardrone/togglecam");
	client_2 = nh_.serviceClient<std_srvs::Empty>("ardrone/flattrim");

	//Subscribe
	summit_position_ = nh_.subscribe(summit_position_topic, 1, &ARDroneJoy::tracking, this);

    int i = 0;
    int j = 0;
    while(i < 2)
    {
        ROS_ERROR("%d",i);
        tf_listener.waitForTransform("/quadrotor/base_link","/image_plane", ros::Time(0), ros::Duration(3.0));
        try{
            tf_listener.lookupTransform("/quadrotor/base_link","/image_plane",ros::Time(0), transform);
            i=3;
            t_ref = ros::Time::now().toSec();
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            i++;
        }

        tf_listener.waitForTransform("/map","/image_plane", ros::Time(0), ros::Duration(3.0));
        try{
            tf_listener.lookupTransform("/map","/image_plane",ros::Time(0), transform_world);
            j=3;
            t_ref = ros::Time::now().toSec();
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            j++;
        }
    }
}


void  ARDroneJoy::tracking(const geometry_msgs::PoseStamped& centroid){

    ROS_INFO("LOCALIZADO");
    tf::Vector3  pose_base_link, pose_in_world;
    tf::Vector3  point(centroid.pose.position.x,centroid.pose.position.y,centroid.pose.position.z);

    //Intento con transformPoint =>
    /*geometry_msgs::PointStamped pose_cam;
      pose_cam.header.frame_id = "/image_plane";
      //we'll just use the most recent transform available for our simple example
      pose_cam.header.stamp = ros::Time();
      pose_cam.point.x = centroid.pose.position.x;
      pose_cam.point.y = centroid.pose.position.y;
      pose_cam.point.z = centroid.pose.position.z;
    ROS_INFO("El punto (%f,  %f , %f)  en metros respecto a la camara",pose_cam.point.x, pose_cam.point.y, pose_cam.point.z);
    geometry_msgs::PointStamped pose_map;*/
    //tf_listener.transformPoint("/image_plane",ros::Time(0), pose_cam,"/map",pose_map);

    //Recalcular transformaciones del frame dinámico /image_plane respecto del frame estático /map
    tf_listener.waitForTransform("/map","/image_plane", ros::Time(0), ros::Duration(0.5)); //0.05
    tf_listener.lookupTransform("/map","/image_plane",ros::Time(0), transform_world);

    pose_base_link = transform * point;
    pose_in_world = transform_world * point;

    ROS_INFO("El punto (%f,  %f , %f)  en metros respecto a la camara",point.getX(),point.getY(), point.getZ());
    ROS_INFO("SummitXL Position: (%f,  %f , %f)  en metros respecto del quadrotor", pose_base_link.getX(), pose_base_link.getY(), pose_base_link.getZ());
    ROS_INFO("SummitXL Position: (%f,  %f , %f)  en metros respecto del mundo", pose_in_world.getX(), pose_in_world.getY(), pose_in_world.getZ());

    ROS_INFO("Tiempo desde comienzo = %.5f", ros::Time::now().toSec() - t_ref);

    // REGULADOR P - proporcional al error ///////////////////////////////////////
    //vel.linear.x = 0.57*pose_base_link.getX();//calculo de K_c
    //vel.linear.x = -0.03;//default -vx of the quadrotor //
    //vel.linear.y = 0.56*pose_base_link.getY();

    /*vel.linear.x = 0.3*current_vel*pose_base_link.getX();
    vel.linear.y = 0.3*current_vel*pose_base_link.getY();*/
    //vel.linear.x = transform.getOrigin().x(),transform.getOrigin().x());
    //////////////////////////////////////////////////////////////////////////////

    //REGULADOR PI - proporcional a la integral del error ////////////////////////
    sumatorio_error_x += pose_base_link.getX(); //Sistema simétrico
    sumatorio_error_y += pose_base_link.getY();

    //vel.linear.x = 0.25*current_vel*pose_base_link.getX() + 0.02*current_vel*sumatorio_error_x ;
    //vel.linear.y = 0.25*current_vel*pose_base_link.getY() + 0.02*current_vel*sumatorio_error_x ;
    //////////////////////////////////////////////////////////////////////////////

    //REGULADOR PID - añadimos proporcionalidad ante la derivada del error ///////
    derror_x= pose_base_link.getX() - position_x0;
    derror_x= pose_base_link.getY() - position_y0;

    vel.linear.x = 0.35*pose_base_link.getX() + 0.0007*sumatorio_error_x + 0.005*derror_x;//bien
    vel.linear.y = 0.35*pose_base_link.getY() + 0.0007*sumatorio_error_x + 0.005*derror_y;

    //vel.linear.x = 0.35*current_vel*pose_base_link.getX() + 0.003*current_vel*sumatorio_error_x + 0.025*current_vel*derror_x;
    //vel.linear.y = 0.35*current_vel*pose_base_link.getY() + 0.003*current_vel*sumatorio_error_x + 0.025*current_vel*derror_y;

    //Estimación de Ziegler-Nichols
    //vel.linear.x = 0.342*current_vel*pose_base_link.getX() + 0.342*current_vel*sumatorio_error_x + 0.0427*current_vel*derror_x;
    //vel.linear.y = 0.336*current_vel*pose_base_link.getY() + 0.388*current_vel*sumatorio_error_x + 0.0726*current_vel*derror_y;

    position_x0 = pose_base_link.getX();
    position_y0 = pose_base_link.getY();
    //////////////////////////////////////////////////////////////////////////////

    geometry_msgs::PoseStamped real_centroid; //posicion del centroide respecto del quadrotor base_link
        real_centroid.header.stamp= ros::Time::now();
        real_centroid.header.seq = ros::Time::now().toSec() - t_ref;
        real_centroid.header.frame_id = "/quadrotor/base_link";
        real_centroid.pose.orientation.w=1;
        real_centroid.pose.orientation.x=0;
        real_centroid.pose.orientation.y=0;
        real_centroid.pose.orientation.z=0;
        real_centroid.pose.position.x=pose_base_link.getX();
        real_centroid.pose.position.y=pose_base_link.getY();
        real_centroid.pose.position.z=pose_base_link.getZ();


    geometry_msgs::PoseStamped estimated_centroid_in_world; //posicin del centride de la etimacion en el mundo
        estimated_centroid_in_world.header.stamp= ros::Time::now();
        estimated_centroid_in_world.header.seq = ros::Time::now().toSec() - t_ref;
        estimated_centroid_in_world.header.frame_id = "/map";
        estimated_centroid_in_world.pose.orientation.w=1;
        estimated_centroid_in_world.pose.orientation.x=0;
        estimated_centroid_in_world.pose.orientation.y=0;
        estimated_centroid_in_world.pose.orientation.z=0;
        estimated_centroid_in_world.pose.position.x=pose_in_world.getX();
        estimated_centroid_in_world.pose.position.y=pose_in_world.getY();
        estimated_centroid_in_world.pose.position.z=pose_in_world.getZ();

    ROS_INFO("Sending Velocity:  (%f,  %f)",vel.linear.x, vel.linear.y);
    ROS_INFO("Acumulated errors:  (%f,  %f)\n\n",sumatorio_error_x, sumatorio_error_y);

    position_quad_base_.publish(real_centroid);
    estimated_summit_position_.publish(estimated_centroid_in_world);
    vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ArDrone_Gamepad");
    ARDroneJoy ardrone_joy;
    ros::Duration(2).sleep();
    ROS_INFO("About to start sequence");
    ros::spin();
}

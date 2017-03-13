#include <ros/ros.h>  //Librería de funciones del sistema ROS
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>   //Librería "Twist" del package "geometry_msgs"
#include "/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/srv_gen/cpp/include/gazebo_msgs/GetModelState.h"
#include "/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/msg_gen/cpp/include/gazebo_msgs/ModelState.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>


//!implementacion de la clase

class ARV
{
public:
  
	
  ARV();

  float x1, x2, y1, y2;
	

private:
	ros::NodeHandle nh;  
	ros::ServiceClient gms1_c, gms2_c;

	gazebo_msgs::GetModelState gms1, gms2;
	
	geometry_msgs::Pose quadrotor_pose, summit_pose;

	ros::Publisher quadrotor_position_;	
	ros::Publisher summit_position_;	
	//geometry_msgs::Twist robot1_twist;
	
};


ARV::ARV()
{	
	gms1.request.model_name="quadrotor";
	gms2.request.model_name="summit_XL";
	
	gms1_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gms2_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	
    quadrotor_position_ = nh.advertise<geometry_msgs::PoseStamped>("/position_quadrotor_modelstate", 1);
    summit_position_ = nh.advertise<geometry_msgs::PoseStamped>("/position_summit_modelstate", 1);

	while(ros::ok()){
		if (gms1_c.call(gms1)){	  //gms now holds the current state
            //ROS_INFO("Position quadrotor service");
		}
		else{
		    ROS_ERROR("Failed to call service 1 ");
		}

		if (gms2_c.call(gms2)){	  //gms now holds the current state
            //ROS_INFO("Position Summit_XL service\n");
		}
		else{
		    ROS_ERROR("Failed to call service 2");
		}
	
		x1 = gms1.response.pose.position.x;
		y1 = gms1.response.pose.position.y;

        ROS_INFO("Quadrotor gazebo position = (%.2f, %.2f) ",x1, y1);
		
		geometry_msgs::PoseStamped quadrotor_real_position;
            quadrotor_real_position.header.stamp= ros::Time::now();
            quadrotor_real_position.header.seq = ros::Time::now().toSec();
            quadrotor_real_position.header.frame_id = "/map";
            quadrotor_real_position.pose.orientation.w=1;
            quadrotor_real_position.pose.orientation.x=0;
            quadrotor_real_position.pose.orientation.y=0;
            quadrotor_real_position.pose.orientation.z=0;
            quadrotor_real_position.pose.position.x=x1;
            quadrotor_real_position.pose.position.y=y1;
            quadrotor_real_position.pose.position.z=5.9;

		x2 = gms2.response.pose.position.x;
		y2 = gms2.response.pose.position.y;

        ROS_INFO("Summit gazebo position = (%.2f, %.2f) \n\n",x2, y2);

		geometry_msgs::PoseStamped summit_real_position;
            summit_real_position.header.stamp= ros::Time::now();
            summit_real_position.header.seq = ros::Time::now().toSec();
            summit_real_position.header.frame_id = "/map";
            summit_real_position.pose.orientation.w=1;
            summit_real_position.pose.orientation.x=0;
            summit_real_position.pose.orientation.y=0;
            summit_real_position.pose.orientation.z=0;
            summit_real_position.pose.position.x=x2;
            summit_real_position.pose.position.y=y2;
            summit_real_position.pose.position.z=0;

            quadrotor_position_.publish(quadrotor_real_position);
            summit_position_.publish(summit_real_position);

	}
	
	
}



int main(int argc, char** argv){

	ros::init(argc, argv, "ARV");
	ARV getmodelstate;

	ros::spin();
}


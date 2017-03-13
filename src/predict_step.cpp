#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <string>
#include "/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/srv_gen/cpp/include/gazebo_msgs/GetModelState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <newmat/newmat.h>  // para matrices
#include <newmat/newmatap.h>
#include "visualization_msgs/Marker.h"
#include "ros/callback_queue.h"
#define MAX 4;

using namespace  NEWMAT;

class kalman {


public:

	kalman();

	
	void kal_predict(int T, ColumnVector x_inic);
	void kal_update(ColumnVector Z);
	void kal_correct();
	void Callback_pos(const geometry_msgs::PoseStampedConstPtr& Pos);
	void inic();
	
	ColumnVector x;

	ColumnVector l; // Donde guardamos el primer valor obtenido
	ColumnVector m; // Donde guardamos la medicion en estructura matricial
	
  	
	Matrix P; Matrix Pl;
	//float P[MAX][MAX]; 
	// Covarianza del Error (prediccion ->) P = A*P*At + Q // (correccion ->) P = (I - K*H)P	

	//VISUALIZACION
	ros::Publisher          pos_mark;
	//PUBLICACION
	ros::Publisher		 pos_pub;
	//COSAS
	float AT; // cada cuanto tiempo 
	bool flag, step;
	float t0, t;
	string frame;
	int contadorcito;
	float h;
    float modz;
    bool parado;

private:  //Debemos hacer todas la operaciones en funciones para poder hacerlo privado y NO usarlo en el main.
	
	//ROS
	ros::NodeHandle      	       n;
	ros::Publisher     	pub_pos2;
	ros::Subscriber         quad_sub;
	

	//MATRICES

	Matrix I;
        
	Matrix A;
	
	Matrix Q;	
	
	Matrix R;
	
	Matrix K;
		
	Matrix H;
		
	ColumnVector Y;
	
	Matrix S;
	
    ColumnVector Z, Z0;

	float z [4]; // medicion

        float x0, y0; // hay q inicializarlo

};



kalman::kalman() {                //Construstor

/////////////////ROS////////////////////

 quad_sub = n.subscribe("/find_summit_final/Estimated_position",1, &kalman::Callback_pos, this); //Suscripcion a la posicion que manda el quadrotor!
 pos_mark = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 pos_pub  = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("Future_position", 1);

 t0 = ros::Time::now().sec + (ros::Time::now().nsec)*1E-9;
 t = t0;
 x0 = y0 = 0;

/////////////COSAS/////////////
 
flag = false;

contadorcito = 0;
	
//////////////MATRICES//////////////////
   
    I = IdentityMatrix(4);

    P = Matrix(4, 4); Pl = Matrix(4, 4);

    R = Matrix(4, 4);
    H = Matrix(4, 4);
    
    K = Matrix(4, 4);   
    Y = ColumnVector(4); 


    P = I; Pl= I; 

    R << 0.09 << 0 << 0 << 0 <<
         0 << 0.09 << 0 << 0 <<
         0 << 0 << 0.36 << 0 <<
         0 << 0 << 0 << 0.36;
    H = I;

    K = I;
    Y << 0 <<
         0 <<
         0 <<
         0;


///// Entrada por pantalla ///////                                                  ->  HAY QUE CORREGIR PARA QUE VALGA CUALQUIER VALOR  <-

        ROS_INFO("\n introduce cada cuanto tiempo realizas la prediccion");
	cin >> AT;

   }

void kalman::kal_predict(int T, ColumnVector x_inic){ //T:cada cuanto tiempo se predice, x_inic a partir de que valor se predice.

    A = Matrix(4, 4);
    Q = Matrix(4, 4);
    
    A << 1 << 0 << T << 0 <<
         0 << 1 << 0 << T <<
         0 << 0 << 1 << 0 <<
         0 << 0 << 0 << 1;


    Q << 0.16 * T << 0 << 0 << 0 <<
         0 << 0.16 * T << 0 << 0 <<
         0 << 0 << 0.2025 * T << 0 <<
         0 << 0 << 0 << 0.2025 * T;


P = A*P*A.t() + Q;
if(parado) x = x_inic;
else x = A*x_inic;

}


void kalman::kal_update(ColumnVector Z){

modz = (Z-Z0).NormFrobenius();
if(modz <= 1.5){
    Y << 0 <<
         0 <<
         0 <<
         0;

    parado = true;
}
else{
    Y = Z  - H*x; //lo que introduzcamos en forma de matriz
    parado = false;
}

S = H*P*H.t()+R;
K = P*H.t()*S.i(); 

//x = x + K*Y;      // Valor predecido!  ******
P = (I - K*H)*P;
Z0 = Z;
}

void kalman::kal_correct(){

x = x + K*Y; // Valor predecido!  ******

ROS_INFO("\n Pos Predicha x: %f, %f, %f, %f", x.element(0), x.element(1), x.element(2), x.element(3) );

}

void kalman::Callback_pos(const geometry_msgs::PoseStampedConstPtr& Pos) // Obtencion de la posicion y convertirla al vector de medicion
{
  
  h = Pos->pose.position.z;
  frame = Pos->header.frame_id; 
  t = Pos->header.stamp.sec + Pos->header.stamp.nsec*1E-9;

	ROS_INFO("\n TIEMPO: %f", t-t0);

  m = ColumnVector(4);

	 z[0] = Pos->pose.position.x;
    	 z[1] = Pos->pose.position.y;

	if (t-t0 >= AT){  //aki regulamos el tiempo de medicion

    	             
   	 z[2] = (z[0]-x0)/(t-t0);
  	 z[3] = (z[1]-y0)/(t-t0);
      
	 
 	 x0 = z[0];
 	 y0 = z[1];
	 t0 = t;
	
	 step = true;

	 }

         else if (contadorcito == 0) {

         z[2] = z[3] = 0;
		
	 contadorcito = contadorcito +1;
	 step = false;
	 }

 m.element(0) = z[0];
 m.element(1) = z[1];
 m.element(2) = z[2];
 m.element(3) = z[3];


 flag = true;
 

ROS_INFO("\n medida: %f, %f, %f, %f", m.element(0), m.element(1), m.element(2), m.element(3));

}

void kalman::inic(){

x = ColumnVector(4);
l = ColumnVector(4);
Z0 = ColumnVector(4);
x << z[0] <<
     z[1] <<
     z[2] <<
     z[3];

Z0 << z[0] <<
      z[1] <<
      z[2] <<
      z[3];


ROS_INFO("\n Posicion inicial x: %f, %f, %f, %f", x.element(0), x.element(1), x.element(2), x.element(3) );

}


int main(int argc, char **argv){
	
  ros::init(argc, argv, "Predict_Step");

  kalman Pos_Kal; // Inicializacion
  while (Pos_Kal.flag == false ){ros::spinOnce();} //Espera hasta tener un valor que incluir en z
  Pos_Kal.inic(); //OK
	sleep(3);

  while (ros::ok()){
	
	//VISUALIZACION

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/image_plane";
		marker.header.stamp = ros::Time::now();
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.ns = "future_pos";
		marker.id = 0;
		marker.color.g = 1.0f;
   		marker.color.a = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;

	//PUBLICACION

		geometry_msgs::PoseWithCovarianceStamped poswithcov;
		poswithcov.header.frame_id = "/image_plane";
		poswithcov.header.stamp = ros::Time::now();

	//PREDICCION
	
		Pos_Kal.kal_predict(Pos_Kal.AT, Pos_Kal.m);
		Pos_Kal.kal_correct();
			
			//Publicaciones
			marker.pose.position.x = Pos_Kal.x.element(0);
			marker.pose.position.y = Pos_Kal.x.element(1);
			marker.pose.position.z = Pos_Kal.h;
			
			Pos_Kal.pos_mark.publish(marker);

			poswithcov.pose.pose.position.x = Pos_Kal.x.element(0);
			poswithcov.pose.pose.position.y = Pos_Kal.x.element(1);
			//poswithcov.pose.covariance[0] = Pos_Kal.P(0, 0);
			//poswithcov.pose.covariance[7] = Pos_Kal.P(1, 1);
			
			Pos_Kal.pos_pub.publish(poswithcov);			


	//CORRECCION
	
          	Pos_Kal.step = false;
	 	while (Pos_Kal.step == false ){
 		ros::spinOnce();}
	
  		Pos_Kal.kal_update(Pos_Kal.m);

	}


  }




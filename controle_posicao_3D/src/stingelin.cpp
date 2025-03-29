#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
 
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include "time.h"
#include <ctime>
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <conio.h>


#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>

using namespace std;

using namespace std::chrono;
 
turtlesim::Pose feedback;
turtlesim::Pose  _feedback;
turtlesim::Pose  _feedback_;
turtlesim::Pose  _feedback__;

turtlesim::Pose feed;
turtlesim::Pose  _feed;
turtlesim::Pose  _feed_;
turtlesim::Pose  _feed__;

nav_msgs::Odometry odom;
gazebo_msgs::ModelState model_state;
geometry_msgs::Pose poseArtag;

fstream outFile;


double Odom_yaw;    //odometry orientation (yaw)
double yaw2;
double temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8;

//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v1                    //
//****************************************************************//
tf::Pose pose;
double x=0,y=0,theta, x_ant, y_ant, delta, travelled = 0;
geometry_msgs::Twist msg;	
bool ori_ok = false, pos_ok = false;

bool flag_x = false;
bool flag_y = false;
float distx_R1 = 99;
float distx_R2 = 99;

float disty_R1 = 99;
float disty_R2 = 99;
float efeito_derivativo = 0;
float planned_angular = 0;
float planned_linear = 0;

float posdesejada[2], oridesejada, error_dist_robo=99,erropos=99, erroorie=99, erropos_1, erropos_2;
float tolerance_orie = 0.05, tolerance_pos = 0.01, tolerance_min = 0.01;
//float tolerance_orie = 99, tolerance_pos = 99, tolerance_min = 99;
float angulo;

double dist_obs, ang_obs;
bool is_obs = false;


bool setpointR1CoordY_metros = false;
bool setpointR1CoordY_centimetros = false;
bool setpointR1CoordY_milimetros = false;

bool setpointR2CoordY_metros = false;
bool setpointR2CoordY_centimetros = false;
bool setpointR2CoordY_milimetros = false;


//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v2                  //
//****************************************************************//
tf::Pose _pose;
double _x=0,_y=0,_theta, _x_ant, _y_ant, _delta, _travelled = 0;
geometry_msgs::Twist _msg;	
bool _ori_ok = false, _pos_ok = false;
float _posdesejada[2], _oridesejada, _error_dist_robo=99,_erropos=99, _erroorie=99, _erropos_1, _erropos_2;
float _tolerance_orie = 0.05, _tolerance_pos = 0.1;
float _angulo;

double _dist_obs, _ang_obs;
bool _is_obs = false;



//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v3                  //
//****************************************************************//
tf::Pose _pose_;
double _x_=0,_y_=0,_theta_,_theta__, _x_ant_, _y_ant_, _delta_, _travelled_ = 0;
geometry_msgs::Twist _msg_;	
bool _ori_ok_ = false, _pos_ok_ = false;
float _posdesejada_[2], _oridesejada_, _erropos_=99, _erroorie_=99, _erropos_1_, _erropos_2_;
float _tolerance_orie_ = 0.05, _tolerance_pos_ = 0.1;
float _angulo_;

double _dist_obs_, _ang_obs_;
bool _is_obs_ = false;






float dcml_coordX = 0.0;
float dcml_coordY = 0.0;

using namespace std;

//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - R1                 //
//****************************************************************//


//Callback da Odometria.
void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{

	tf::poseMsgToTF(msg->pose.pose, pose);
  	theta = tf::getYaw(pose.getRotation());

	
	feedback.x = - msg->pose.pose.position.x;
	feedback.y = msg->pose.pose.position.y;

	temp1 = msg->pose.pose.position.x;

	
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  feedback.theta = yaw + M_PI/2;
  
}



//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - v2                 //
//****************************************************************//

void cb(ar_track_alvar_msgs::AlvarMarkers _msg) {

	    if (!_msg.markers.empty()) {
      tf::Quaternion q(_msg.markers[0].pose.pose.orientation.x, _msg.markers[0].pose.pose.orientation.y, _msg.markers[0].pose.pose.orientation.z, _msg.markers[0].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double _roll, _pitch, _yaw;
      m.getRPY(_roll, _pitch, _yaw);
      //ROS_INFO("_roll, _pitch, _yaw=%1.2f  %1.2f  %1.2f", _roll, _pitch, _yaw);
	  
	  	_feedback.x = _msg.markers[0].pose.pose.position.x;
		_feedback.y = _msg.markers[0].pose.pose.position.y;

		temp2 = _msg.markers[0].pose.pose.position.x;
	  
	  _feedback.theta = _yaw;
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
    } // if

/*
printf("position (x,y,z) = (%f : %f : %f)  \n", _msg.markers[0].pose.pose.position.x, _msg.markers[0].pose.pose.position.y, _msg.markers[0].pose.pose.position.z);

	tf::poseMsgToTF(_msg.markers[0].pose.pose, pose);
  	_theta = tf::getYaw(pose.getRotation());
	Odom_yaw = tf::getYaw(_msg.markers[0].pose.pose.orientation);
	
	_feedback.x = _msg.markers[0].pose.pose.position.x;
	_feedback.y = _msg.markers[0].pose.pose.position.y;

	temp2 = _msg.markers[0].pose.pose.position.x;

	tf::Quaternion q(_msg.markers[0].pose.pose.orientation.x, _msg.markers[0].pose.pose.orientation.y,
                  _msg.markers[0].pose.pose.orientation.z, _msg.markers[0].pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double _roll, _pitch, _yaw;
	m.getRPY(_roll, _pitch, _yaw);

	//_feedback.theta = req.markers[0].pose.pose.orientation.y + M_PI/2;
	_feedback.theta = Odom_yaw;
	_feedback__.theta = _msg.markers[0].pose.pose.orientation.y + M_PI/2;
*/



	poseArtag.position.x = _msg.markers[0].pose.pose.position.x;
	poseArtag.position.y = _msg.markers[0].pose.pose.position.y;

	//gazebo_msgs::ModelState model_state;
// This string results from the spawn_urdf call in the box.launch file argument: -model box
	model_state.model_name = std::string("tb3_2");
	model_state.pose = poseArtag;
	model_state.reference_frame = std::string("world");

	


}



void marker_sub_CB(const visualization_msgs::Marker &marker_sub)
{

	//target_odom_point_tmp.pose.position=marker_sub.pose.position;
	//target_odom_point_tmp.pose.orientation=marker_sub.pose.orientation;
/*
	tf::Quaternion quat;
		double roll, pitch, yaw;
		target_odom_point.pose.orientation.x=0;
		target_odom_point.pose.orientation.y=0;

		tf::quaternionMsgToTF(target_odom_point.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		_theta__ = tf::getYaw(pose.getRotation());
		yaw +=1.5708;//旋转90
		//target_odom_point.pose.position.x -= keep_distance*cos(yaw);
		//target_odom_point.pose.position.y -= keep_distance*sin(yaw);

		target_odom_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		target_odom_point.pose.position.z = 0;
	

		//req.markers[0]

		//_feedback_.x = target_odom_point_tmp.pose.position.x;
		//_feedback_.y = target_odom_point_tmp.pose.position.y;
		//robot.pose.orientation.x = robot.pose.orientation.y = robot.pose.orientation.z = 0;
*/
		
		/*
		if(marker_sub.pose.position.x < lastX){
					
			posePlanned.position.x = lastX;

		}else{
						
			posePlanned.position.x = marker_sub.pose.position.x;
		}
		*/

		//posePlanned.position.x = marker_sub.pose.position.x;


		//posePlanned.position.y = marker_sub.pose.position.y;

		/*
		if(marker_sub.pose.position.y < lastY){
					
			posePlanned.position.y = lastY;

		}else{
						
			posePlanned.position.y = marker_sub.pose.position.y;
		}
		*/
		//posePlanned.orientation.x = marker_sub.pose.orientation.x;
		//posePlanned.orientation.y = marker_sub.pose.orientation.y;
		//posePlanned.orientation.z = marker_sub.pose.orientation.z;
		//posePlanned.orientation.w = marker_sub.pose.orientation.w;


		//temp1 = posePlanned.orientation.x;
		//temp2 = posePlanned.orientation.y;
		//temp3 = posePlanned.orientation.z;
		//temp4 = posePlanned.orientation.w;

		//lastX = marker_sub.pose.position.x;
		//lastY = marker_sub.pose.position.y;

		//_feedback__.theta = yaw + M_PI/2;


		//posePlanned.orientation.z = pitch;
		//posePlanned.orientation.z = target_odom_point_tmp.pose.orientation.w;
		
		//gazebo_msgs::ModelState model_state;
		// This string results from the spawn_urdf call in the box.launch file argument: -model box

		//tf::poseMsgToTF(marker_sub->pose, pose);
		//_theta__ = tf::getYaw(pose.getRotation());
		//Odom_yaw = tf::getYaw(marker_sub.pose.orientation);

		//_feedback_.x = _msg_->pose.pose.position.x;
		_feed__.x = - marker_sub.pose.position.z;
		_feed__.y = marker_sub.pose.position.x;

		_feedback__.x = marker_sub.pose.position.z;
		_feedback__.y = marker_sub.pose.position.x;
		
		temp4 = marker_sub.pose.position.y;


		tf::Quaternion q(marker_sub.pose.orientation.x, marker_sub.pose.orientation.y,
			marker_sub.pose.orientation.z, marker_sub.pose.orientation.w);

		tf::Matrix3x3 m(q);
		double _roll__, _pitch__, _yaw__;
		m.getRPY(_roll__, _pitch__, _yaw__);

		_feedback__.theta = _yaw__ + M_PI/2;

		
		//tf::Quaternion q(marker_sub.pose.orientation.x, marker_sub.pose.orientation.y,
		//				marker_sub.pose.orientation.z, marker_sub.pose.orientation.w);
		
		//tf::Matrix3x3 m(q);
		//double _roll__, _pitch__, _yaw__;
		//m.getRPY(_roll__, _pitch__, _yaw__);

		


		//lastX = _yaw__ + M_PI/2;

		//_feedback__.theta = _yaw__ + M_PI/2;

	
		//if(lastX < 2.27){
					
			//_feedback__.theta = 2.27942;
			
		//}else{
			
			//_feedback__.theta = lastX;
						
		//}

	
	  
}


//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - v3                 //
//****************************************************************//

//Callback da Odometria.
void _subCallback_odom_(const nav_msgs::Odometry::ConstPtr& _msg_)
{


   	tf::poseMsgToTF(_msg_->pose.pose, pose);
  	_theta_ = tf::getYaw(pose.getRotation());

	_feed_.x = - _msg_->pose.pose.position.z;
	_feed_.y = _msg_->pose.pose.position.y;

	_feedback_.x = - _msg_->pose.pose.position.x;
	_feedback_.y = _msg_->pose.pose.position.y;

	temp3 = _msg_->pose.pose.position.x;
	
  tf::Quaternion q(_msg_->pose.pose.orientation.x, _msg_->pose.pose.orientation.y,
                  _msg_->pose.pose.orientation.z, _msg_->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double _roll_, _pitch_, _yaw_;
  m.getRPY(_roll_, _pitch_, _yaw_);

  _feedback_.theta = _yaw_ + M_PI/2;
	  
}


float ajuste1(float k){

float efeito_derivativo1 = 0;
bool efeito_derivativoNAN1 = false;


	efeito_derivativo1 = ((1/k))*-1;
	efeito_derivativoNAN1 = isnan(efeito_derivativo1) ? true : false;
					
		if(efeito_derivativoNAN1){
					
			efeito_derivativo1 = 0;
			return efeito_derivativo1;

		}else{
						
			efeito_derivativo1 = efeito_derivativo1-4;
			return efeito_derivativo1;
		}

}


float ajuste2(float k){

float efeito_derivativo2 = 0;
bool efeito_derivativoNAN2 = false;


	efeito_derivativo2 = ((1/k))*-1;
	efeito_derivativoNAN2 = isnan(efeito_derivativo2) ? true : false;
					
		if(efeito_derivativoNAN2){
					
			efeito_derivativo2 = 0;
			return efeito_derivativo2;

		}else{
			
			efeito_derivativo2 = efeito_derivativo2+4;
			return efeito_derivativo2;
						
		}

		
}


float ajuste3(float k){

	float efeito_derivativo3 = 0;
	bool efeito_derivativoNAN3 = false;
	
	
		efeito_derivativo3 = abs(1/k);
		efeito_derivativoNAN3 = isnan(efeito_derivativo3) ? true : false;
						
			if(efeito_derivativoNAN3){
						
				efeito_derivativo3 = 0;
				return efeito_derivativo3;
	
			}else{
				
				efeito_derivativo3 = efeito_derivativo3+4;
				return efeito_derivativo3;
							
			}
	
			
	}


int main(int argc, char **argv)
{

//****************************************************************//
//	   					CONFIGURAÇÃO DO ROS 		              //
//****************************************************************//
ros::init(argc, argv, "stingelin");

ros::NodeHandle n;
ros::Publisher pub = n.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel1", 1000);
ros::Subscriber sub = n.subscribe("tb3_0/pose1", 1000, subCallback_odom);


ros::NodeHandle _n;
ros::Publisher _pub = _n.advertise<geometry_msgs::Twist>("cmd_vel2", 1000);
ros::Publisher gazebo_model_state_pub = _n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
ros::Subscriber _sub = _n.subscribe("ar_pose_marker", 1, &cb);


ros::NodeHandle _n_;
ros::Publisher _pub_ = _n_.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel3", 1000);
ros::Subscriber _sub_ = _n_.subscribe("tb3_1/odom", 1000, _subCallback_odom_);



ros::NodeHandle _n__;
//ros::Publisher _pub_ = _n_.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel3", 1000);
//ros::Publisher gazebo_model_state_pub = _n_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

ros::Subscriber marker_sub = _n__.subscribe("/visualization_marker", 1000, marker_sub_CB);
//ros::Subscriber marker_sub=n.subscribe("/visualization_marker",10,marker_sub_CB);


ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = current_time;

ros::Rate loop_rate(10);





//****************************************************************//
//			    PROMPT SETPOINT (tolerância de desvio)	
//
// Código de planejamento de trajetória, "caminho feliz", Coppeliasim,
// informa o parametro para se saber os percentuais de desvios do 
// virtual e real em relação ao planejado, então os percentuais
// serão aplicados em ambos os robos, como aumento ou diminuição da 
// velocidade para encontrar a centralidade do caminho planejado.
//****************************************************************//




//****************************************************************//
//			 			CONTROLE DE POSIÇÃO (DCML)	 	                  //
//
// Aplicar o DCML obtendo dados cinemáticos das 3 entidades, 
// planejador, virtual e real, usar saida percentual do DCML para informar
// o parametro de ajuste para o controle de posição e
// plotar grafico demonstrando o efeito da DCML sobre o controlador 
// fuzzy e pid.
//****************************************************************//

	



		//cout << "Digite a posicao R1\nX>>";
    	//cin >> posdesejada[0];
		//cout << "Digite a posicao R2\nX>>";
 		//cin >> _posdesejada[0];

		posdesejada[0] = 0.90;
		posdesejada[1] = 0.90;
		

    	//cout << "Y para R1>>";
    	//cin >> posdesejada[1];
		//cout << "Y para R2>>";
		//cin >> _posdesejada[1];

		_posdesejada[0] = 0.90;
		_posdesejada[1] = 0.90;

    	//ros::spinOnce();

				// current date/time based on current system
				time_t now = time(0);
				
				// convert now to string form
				char* dt = ctime(&now);

				//cout << "The local date and time is: " << dt << endl;

				//std::chrono::seconds
				auto tStartSteady = std::chrono::steady_clock::now();
				std::time_t startWallTime = system_clock::to_time_t(system_clock::now());
				
				//std::cout << "Time start = " << std::ctime(&startWallTime) << " \n";
				//testFunction();
				auto tEndSteady = std::chrono::steady_clock::now();
				nanoseconds diff = tEndSteady - tStartSteady;
				std::time_t endWallTime = system_clock::to_time_t(system_clock::now());

				//std::cout << "Time end = " << std::ctime(&endWallTime) << " \n";
				//std::cout << "Time taken = " << diff.count() << " ns";

				auto t_start = std::chrono::high_resolution_clock::now();
    			// the work...
				auto t_end = std::chrono::high_resolution_clock::now();
				//double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();


				auto start = steady_clock::now() ;

				//std::cin.get() ; // whatever is to be timed

				auto end = steady_clock::now() ;

				std::cout << duration_cast<milliseconds>(end-start).count() << " milliseconds\n"
          			<< duration_cast<seconds>(end-start).count() << " seconds\n" ;

	

		outFile.open("stingelin.txt", ios::app);

		//pos_ok = false;
		//ori_ok = false;
		
		while (/*_feedback.y <= 0.90 &&*/ feedback.y <= 0.90) {
 
				auto end = steady_clock::now() ;
				double elapsed_time_ms = duration_cast<seconds>(end - start).count();
				
				//while (ros::ok()){
					gazebo_model_state_pub.publish(model_state);
					//}
	                
		            //dcml_coordX = ((_feedback_.x-feedback.x)-(_feedback_.x-_feedback.x)/(log(_feedback_.x-feedback.x/_feedback_.x-_feedback.x)));
                    //printf("dcml_coordX: %.2f m\n",dcml_coordX);
					float CidealX = temp4;
					float CvirtualX = temp1;
  					float CfisicoX = temp2;
  					
					float K2X = CidealX-CvirtualX;
  					float L2X = CidealX-CfisicoX;

					dcml_coordX  = (K2X-L2X/log(K2X/L2X));
					printf("dcml_coordX: %.2f m\n",dcml_coordX);







					float CidealX1 = temp4;
					float CvirtualX1 = temp3;
  					float CfisicoX1 = temp2;
  					
					float K2X1 = CidealX1-CvirtualX1;
  					float L2X1 = CidealX1-CfisicoX1;

					float dcml_coordX1  = (K2X1-L2X1/log(K2X1/L2X1));
					planned_angular = dcml_coordX1;
					printf("dcml_coordX1: %.2f m\n",dcml_coordX1);
/*////////////////////////////////////////////////////////////////////////////////////////*/
					float Cideal_ = _feedback__.y;
					float Cvirtual_ = _feedback_.y;
  					float Cfisico_ = _feedback.y;
  					
					float K2_ = Cideal_-Cvirtual_;
  					float L2_ = Cideal_-Cfisico_;

					float dcml_coordY_ = (K2_-L2_/log(K2_/L2_));
					planned_linear = dcml_coordY_;
					printf("dcml_coordY_: %.2f m\n",dcml_coordY_);








					
					float Cideal = _feedback__.y;
					float Cvirtual = feedback.y;
  					float Cfisico = _feedback.y;
  					
					float K2 = Cideal-Cvirtual;
  					float L2 = Cideal-Cfisico;

					float _dcml_coordY = (K2-L2/log(K2/L2));
					efeito_derivativo = _dcml_coordY;
					printf("_dcml_coordY: %.2f m\n",_dcml_coordY);


                    dcml_coordY = ((_feedback_.y-feedback.y)-(_feedback_.y-_feedback.y)/(log(_feedback_.y-feedback.y/_feedback_.y-_feedback.y)));
					printf("dcml_coordY: %.2f m\n",dcml_coordY);
		

					setpointR1CoordY_metros = dcml_coordY > 20 && dcml_coordY < 200 ? true : false;
					//setpointR1CoordY_metros = std::max(200.0, std::min(20.0, setpointR1CoordY_metros));
					//printf("setpointR1CoordY_metros:",setpointR1CoordY_metros);

					setpointR1CoordY_centimetros = dcml_coordY > 201 && dcml_coordY < 800 ? true : false;
					//setpointR1CoordY_centimetros = std::max(800.0, std::min(201.0, setpointR1CoordY_centimetros));
					//setpointR1CoordY_milimetros = std::max(200000.0, std::min(801.0, setpointR1CoordY_milimetros));

					setpointR1CoordY_milimetros = dcml_coordY > 801 && dcml_coordY < 200000 ? true : false;



					setpointR2CoordY_metros = dcml_coordY < -2 && dcml_coordY > -200 ? true : false;
					//setpointR2CoordY_metros = std::max(-200.0, std::min(-20.0, setpointR2CoordY_metros));
					//printf("setpointR2CoordY_metros:",setpointR2CoordY_metros);

					setpointR2CoordY_centimetros = dcml_coordY < -201 && dcml_coordY > -800 ? true : false;
					//setpointR2CoordY_centimetros = std::max(-800.0, std::min(-201.0, setpointR2CoordY_centimetros));
					//setpointR2CoordY_milimetros = std::max(-200000.0, std::min(-801.0, setpointR2CoordY_milimetros));

					setpointR2CoordY_milimetros = dcml_coordY < -801 && dcml_coordY > -200000 ? true : false;


					disty_R1 = _feedback_.y - feedback.y;
					printf("disty_R1: %.2f m\n",disty_R1);
            		disty_R2 = _feedback_.y - _feedback.y;
					printf("disty_R2: %.2f m\n",disty_R2);
			
					distx_R1 = temp3 - temp1;
					printf("distx_R1: %.2f m\n",distx_R1);
            		distx_R2 = temp3 - temp2;
					printf("distx_R2: %.2f m\n",distx_R2);

					printf("_feedback_.x: %.2f m\n",_feedback_.x);
					printf("_feedback_.y: %.2f m\n",_feedback_.y);
					printf("feedback.x: %.2f m\n",feedback.x);
					
					printf("feedback.y: %.2f m\n",feedback.y);
					printf("_feedback.x: %.2f m\n",_feedback.x);
					
					printf("_feedback.y: %.2f m\n",_feedback.y);
	            //}	
		          //  ros::spinOnce();
            
            //Verifica se há objetos próximos em v1.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.

            error_dist_robo = sqrt(pow(posdesejada[0]-temp4,2)+pow(posdesejada[1]-_feedback__.y,2));
            _error_dist_robo = sqrt(pow(_posdesejada[0]-temp2,2)+pow(_posdesejada[1]-_feedback.y,2));

            //dcml_coordX = ((_feedback_.x-feedback.x)-(_feedback_.x-_feedback.x)/(log(_feedback_.x-feedback.x/_feedback_.x-_feedback.x)))
            flag_x = isnan(dcml_coordX) ? true : false;
            flag_y = isnan(_dcml_coordY) ? true : false;
            
            is_obs = dcml_coordX > 0 ? true : false;
            _is_obs = _dcml_coordY > 0 ? true : false;

					//Verifica se há objetos próximos em v2.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.

			if(/*_feedback.y < _feedback_.y && */_feedback_.y < _feedback__.y){

			if (_is_obs){

				if(_dcml_coordY > 2 && _dcml_coordY < 200){

					msg.linear.x = 0.02;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

					_msg.linear.x = -0.2 + (ajuste1(efeito_derivativo)*0.02);

					_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

					printf("velocidade KP azul (REAL): %.2f m\n",_msg.linear.x);

					}else if(_dcml_coordY > 201 && _dcml_coordY < 800){
				
						msg.linear.x = 0.02;
						
						pub.publish(msg);
						ros::spinOnce();
						loop_rate.sleep();

						_msg.linear.x = -0.2 + (ajuste1(efeito_derivativo)*0.001);

						_pub.publish(_msg);
						ros::spinOnce();
						loop_rate.sleep();

						printf("velocidade KI azul (REAL): %.2f m\n",_msg.linear.x);

						}else if(_dcml_coordY > 801 && _dcml_coordY < 200000){
							msg.linear.x = 0.02;
							
							pub.publish(msg);
							ros::spinOnce();
							loop_rate.sleep();

							_msg.linear.x = -0.2 + (ajuste1(efeito_derivativo)*0.01);

							_pub.publish(_msg);
							ros::spinOnce();
							loop_rate.sleep();

							printf("velocidade KD azul (REAL): %.2f m\n",_msg.linear.x);

								}else{
						
								msg.linear.x = 0.02;
								
								pub.publish(msg);
								ros::spinOnce();
								loop_rate.sleep();

								
								_msg.linear.x = -0.2 + (ajuste1(efeito_derivativo)*0.01);

								_pub.publish(_msg);
								ros::spinOnce();
								loop_rate.sleep();

								printf("velocidade N azul (REAL): %.2f m\n",_msg.linear.x);

								}

			
			}else{
	
				
					if (_dcml_coordY < -2 && _dcml_coordY > -200){

					
						_msg.linear.x = -0.2;

						_pub.publish(_msg);
						ros::spinOnce();
						loop_rate.sleep();

						msg.linear.x = 0.02 + (ajuste2(efeito_derivativo)*0.02);
							
						pub.publish(msg);
						ros::spinOnce();
						loop_rate.sleep();


						}else if(_dcml_coordY < -201 && _dcml_coordY > -800){
					
							_msg.linear.x = -0.2;

							_pub.publish(_msg);
							ros::spinOnce();
							loop_rate.sleep();

							msg.linear.x = 0.02 + (ajuste2(efeito_derivativo)*0.001);
							
							pub.publish(msg);
							ros::spinOnce();
							loop_rate.sleep();

					
							}else if(_dcml_coordY < -801 && _dcml_coordY > -200000){
						
								_msg.linear.x = -0.2;

								_pub.publish(_msg);
								ros::spinOnce();
								loop_rate.sleep();

								msg.linear.x = 0.02 + (ajuste2(efeito_derivativo)*0.01);
							
								pub.publish(msg);
								ros::spinOnce();
								loop_rate.sleep();

							
							}else{
					
							msg.linear.x = 0.02 + (ajuste2(efeito_derivativo)*0.01);
							
							pub.publish(msg);
							ros::spinOnce();
							loop_rate.sleep();

							
							_msg.linear.x = -0.2;
							
							_pub.publish(_msg);
							ros::spinOnce();
							loop_rate.sleep();

							}

				

			}

				}else{
									
					msg.linear.x = 0;
							
					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();

					
					_msg.linear.x = 0;
					
					_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				}
































					//dcml_coordX1
					//dcml_coordY_

								double dx = temp4-temp3;
								double dy = _feedback__.y-_feedback_.y;
								double distance = sqrt(pow(dx,2)+pow(dy,2));

								if((abs(error_dist_robo) > tolerance_pos)){
									
									
									_msg_.linear.x = 0.02 + (ajuste3(planned_linear)*0.01);
							
									_pub_.publish(_msg_);
									ros::spinOnce();
									loop_rate.sleep();

									//_msg_.linear.x = 0.15;
									
									//_pub_.publish(_msg_);
									//ros::spinOnce();
									//loop_rate.sleep();

									/*
									
																		//angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
									//Calcula o setpoint da orientação.
									_angulo_ = atan2(_feedback__.y-_feedback_.y,temp4-temp3);
									
									//Calcula a diferença entre os ângulos.
									_erroorie_ = _angulo_ - _feedback_.theta;

									if (abs(_erroorie_) > tolerance_orie){

										_msg_.angular.z = _erroorie_/(60/10);
										_pub_.publish(_msg_);
									}
									*/
								}else{

									_msg_.linear.x = 0;
									_pub_.publish(_msg_);

									ros::spinOnce();
									loop_rate.sleep();
								}
								
								//do{
								//if(temp3  <= temp4){
								//if((abs(_erroorie_) <= 0.1){
							
									
									
									//angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
									//Calcula o setpoint da orientação.
									_angulo_ = atan2(_feed__.x-_feed_.x, _feed__.y-_feed_.y);
									
									//Calcula a diferença entre os ângulos.
									//_erroorie_ = _angulo_ - _feedback_.theta;
									_erroorie_ = _angulo_ - _feedback_.theta;

									if (abs(_erroorie_) > 0.005){

										//_msg_.angular.z = (_erroorie_/(180/10))*-1;
										_msg_.angular.z = 0.02 + (ajuste3(planned_angular)*0.01);
										_pub_.publish(_msg_);
										
										ros::spinOnce();
										loop_rate.sleep();
									
									
								}else{

									_msg_.angular.z = 0;
									_pub_.publish(_msg_);


								}

/*
							if((abs(temp3) < abs(temp4)) && (abs(_feedback_.y) < abs(_feedback__.y))){
								if((abs(dcml_coordX1) > 0.70)){


									_msg_.linear.x = 0.02 + (ajuste2(efeito_derivativo)*0.01);
									_pub_.publish(_msg_);

									//_msg_.angular.z = 0.02 + (ajuste2(efeito_derivativo)*0.01);
									//_pub_.publish(_msg_);

									ros::spinOnce();
									loop_rate.sleep();
								}else{


									_msg_.angular.z = 0.02 + (ajuste2(efeito_derivativo)*0.01);
									_pub_.publish(_msg_);
									

									ros::spinOnce();
									loop_rate.sleep();
								}
							}else{
								
								_msg_.linear.x = 0;
								_pub_.publish(_msg_);
								_msg_.angular.z = 0;
								_pub_.publish(_msg_);

								ros::spinOnce();
								loop_rate.sleep();
							}
*/









































			if(/*_feedback.y < _feedback_.y && */temp1 < temp4){

			if (is_obs){

			if(distx_R1 > tolerance_pos){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie =  angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
						//msg.angular.z = 0;
                        pub.publish(msg);

				    
                    }


			}
			

			if (distx_R2 > tolerance_pos){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _feedback.theta - _angulo;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
						//_msg.angular.z = 0;
                        _pub.publish(_msg);

				    
                    }


			
				}
			
			}else{

				if(distx_R1 > tolerance_pos){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie = angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
						//msg.angular.z = 0;
                        pub.publish(msg);

				    
                    }


				}
			

				if (distx_R2 > tolerance_pos){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _feedback.theta - _angulo;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
						//_msg.angular.z = 0;
                        _pub.publish(_msg);

				    
                    }


			
				}
				

			}

		}else{
									
			//msg.angular.z = erroorie/(60/10);
			msg.angular.z = 0;
			pub.publish(msg);

			
			//_msg.angular.z = _erroorie/(60/10);
			_msg.angular.z = 0;
			_pub.publish(_msg);

		}











		if(/*_feedback.y < _feedback_.y && */feedback.y < _feedback__.y){
			if (flag_y){
				if(disty_R1 > tolerance_min){

					msg.linear.x = 0.02;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	msg.linear.x = 0;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}
			

			if (disty_R2 > tolerance_min){

				

					_msg.linear.x = -0.2;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	_msg.linear.x = 0;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				
				}
			}
		}else{
									
			msg.linear.x = 0;
					
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();

			
			_msg.linear.x = 0;
			
			_pub.publish(_msg);
			ros::spinOnce();
			loop_rate.sleep();

		}












			if(/*_feedback.y < _feedback_.y && */temp1 < temp4){
			if (flag_x){


				if(distx_R1 > tolerance_min){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie = angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
						//msg.angular.z = 0;
                        pub.publish(msg);

				    
                    }


				}else{
						msg.angular.z = 0;
                        pub.publish(msg);
					}
			

			if (distx_R2 > tolerance_min){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _feedback.theta - _angulo;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
						//_msg.angular.z = 0;
                        _pub.publish(_msg);

				    
                    }


			
				}else{
						_msg.angular.z = 0;
                        _pub.publish(_msg);
					}

				}
			}else{
									
				//msg.angular.z = erroorie/(60/10);
				msg.angular.z = 0;
				pub.publish(msg);
	
				
				//_msg.angular.z = _erroorie/(60/10);
				_msg.angular.z = 0;
				_pub.publish(_msg);
	
			}












			
				pub.publish(msg);
				_pub.publish(_msg);
				ros::spinOnce();
				loop_rate.sleep();
			


				outFile << elapsed_time_ms << "," << temp1 << "," << temp2 << "," << temp3 << "," << temp4 << ","
				<< "######" << "," << feedback.y << "," << _feedback.y << "," << _feedback_.y <<  "," << _feedback__.y <<  ","
				<< "######" << "," << feedback.x << "," << _feedback.x << "," << _feedback_.x <<  "," << _feedback__.x <<  ","
				<< "######" << "," << feedback.theta << "," << _feedback.theta << "," << _feedback_.theta <<  "," << _feedback__.theta <<  ","
				<< "######" << "," << angulo << "," << _angulo << "," << erroorie <<  "," << _erroorie <<  "," << msg.angular.z <<  "," << _msg.angular.z <<  ","
				<< "######" << "," << msg.linear.x << "," << _msg.linear.x << "," << dcml_coordX << "," << _dcml_coordY <<  ","
				<< "######" << "," << dcml_coordX1 << "," << dcml_coordY_ << endl; // << Chega aqui ele não faz nada
		}

			outFile.close();
			
			
			//dcml_coordX1
			//dcml_coordY_
			//system("copy /home/fabiano/catkin_ws/tingelin.txt" "/home/fabiano/backups /y&quot");
	

			char old_name[] = "stingelin.txt";
			char new_name[]=".txt";
			int value;
    		//strcat(dt,new_name);
			value = rename(old_name,strcat(dt,new_name));
			

			//outFile.open("/home/fabiano/stingelin.txt", ios::app);

			//system("pause");
            return 0;
	}
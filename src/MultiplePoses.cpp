#include <iostream>
#include <vector>
#include <aruco/aruco.h>
#include<algorithm>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <new_msgs/MarkerPose.h>
#include <new_msgs/MultipleMarkerPose.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ExtendedTaskHierarchy.h>

using namespace Eigen;

new_msgs::MarkerPose msg1;
new_msgs::MultipleMarkerPose msg2;
new_msgs::MultipleMarkerPose msg3;
std_msgs::Float64MultiArray array;

ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;
ros::NodeHandle *nh;
float minimum;
int minimumid;

std_msgs::Float64MultiArray transformation;
double x,y,z,w,x_cor,y_cor,z_cor;

MatrixXd T_kinect_arm;
bool transformation_init = false;





void randomcall_back(aruco_msgs::MarkerArray::Ptr msg)
{

pub = nh->advertise<new_msgs::MultipleMarkerPose>("/multiplePoseArray", 1);
pub1 = nh->advertise<std_msgs::Float64MultiArray>("/kinect_armtransform", 1);
pub2 = nh->advertise<new_msgs::MultipleMarkerPose>("/marker_armtransform", 1);

Matrix3d R_kinect_marker;
Vector3d t_kinect_marker;
MatrixXd T_kinect_arm(4,4);

MatrixXd T_marker_arm(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;

/*
    R <<  0, 0, -1,
          0, -1, 0,
          -1, 0, 0;
  */        
/*
          R << 0,-1,0,
          0,0,-1,
          1,0,0;
          */
     R2 << 0,1,0,
          -1,0,0,
           0,0,1;

T_marker_arm.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
//t_marker_arm << -0.025,-1.075,-0.05;
t_marker_arm << -0.075,-0.05,-1.075;
//t_marker_arm << -0.075,-1.075,-0.05;
T_marker_arm.block(0,3,3,1) = t_marker_arm;
T_marker_arm.row(3) << 0,0,0,1;

msg2.Multiplemarkers.resize(msg->markers.size());
msg3.Multiplemarkers.resize(msg->markers.size());





for(int i=0;i<msg->markers.size();i++)
{
msg2.header.stamp = ros::Time::now();
msg2.header.frame_id = "kinect2_link";
msg2.Multiplemarkers[i].id =msg->markers[i].id;// msg1.id;
msg2.Multiplemarkers[i].pose = msg->markers[i].pose.pose;//msg1.pose;

for(int j=0;j<msg->markers.size();j++)
{
	x_cor=msg2.Multiplemarkers[j].pose.position.x;
	y_cor=msg2.Multiplemarkers[j].pose.position.y;
	z_cor=msg2.Multiplemarkers[j].pose.position.z;

float distance[5];
int markerId[5];

markerId[j]=msg->markers[j].id;

int Id=msg->markers[j].id;
 distance[j] = sqrt(x_cor*x_cor + y_cor*y_cor + z_cor*z_cor);
 
cout<<"marker id"<<Id<<endl;
cout<<"marker id"<<Id<<"distance"<<distance[0]<<endl;
cout<<"marker id"<<Id<<"distance"<<distance[1]<<endl;


 minimum = distance[0];
 minimumid = msg->markers[0].id;
  
  


     if (distance[j] < minimum || markerId[j] <minimumid)
        {
          minimum  = distance[j];
         
         minimumid = markerId[j];
        }
  }
  
  
int Id=msg->markers[i].id;


// from the array of detected markers calculates the distances and sort the smallest distance



//cout <<"Enter  marker id: \n";
//cin >>Id;

switch ( Id){





case 450:
	//if(msg->markers[i].id == 322){
{

MatrixXd T_marker_arm1(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;


    R <<  0, 0, -1,
          0, 1, 0,
          1, 0, 0;

T_marker_arm1.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
t_marker_arm << -0.075,0.05,-1.075;
T_marker_arm1.block(0,3,3,1) = t_marker_arm;
T_marker_arm1.row(3) << 0,0,0,1;


	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	x=msg2.Multiplemarkers[i].pose.orientation.x;
	y=msg2.Multiplemarkers[i].pose.orientation.y;
	z=msg2.Multiplemarkers[i].pose.orientation.z;
	w=msg2.Multiplemarkers[i].pose.orientation.w;
	/*
	VectorXd quat(4);
	quat << x,y,z,w;
	Matrix3d RR;
	RR = quat2rot(quat);*/
	Quaterniond q(x, y, z, w);
	q.normalize();
	R1 = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 

	MatrixXd T_kinect_marker(4,4); 
	Matrix3d R_kinect_marker;

	R_kinect_marker << R1(0,0),R1(0,1),R1(0,2),
	      R1(1,0),R1(1,1),R1(1,2),
	      R1(2,0),R1(2,1),R1(2,2);
	      

	     
	T_kinect_marker.block(0,0,3,3) = R_kinect_marker;
	//T_kinect_marker.block(0,0,3,3) = R1;
	Vector3d t_kinect_marker( x_cor,y_cor,z_cor);
	T_kinect_marker.block(0,3,3,1) = t_kinect_marker;
	T_kinect_marker.row(3) << 0,0,0,1;


	T_kinect_arm = T_kinect_marker*T_marker_arm1;
//cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	
	if(minimumid ==450)
	{
	pub1.publish(array);
	transformation_init = true;
	
	}
	
}

break;
	
case 300:
	//if(msg->markers[i].id == 322){
{
MatrixXd T_marker_arm2(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;

/*
    R <<   -1,  0, 0,
           0,  0, 1,
           0,  -1, 0 ;*/
          
   R <<   -1,  0, 0,
           0,  -1, 0,
           0,  0, 1 ;
 

T_marker_arm2.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
t_marker_arm << -0.975,-0.05,-0.075;
T_marker_arm2.block(0,3,3,1) = t_marker_arm;
T_marker_arm2.row(3) << 0,0,0,1;

	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	x=msg2.Multiplemarkers[i].pose.orientation.x;
	y=msg2.Multiplemarkers[i].pose.orientation.y;
	z=msg2.Multiplemarkers[i].pose.orientation.z;
	w=msg2.Multiplemarkers[i].pose.orientation.w;
	
	
	Quaterniond q(x, y, z, w);
	q.normalize();
	R1 = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 

	MatrixXd T_kinect_marker(4,4); 
	Matrix3d R_kinect_marker;

	R_kinect_marker << R1(0),R1(1),R1(2),
	      R1(3),R1(4),R1(5),
	      R1(6),R1(7),R1(8);
	  
	     
	T_kinect_marker.block(0,0,3,3) = R_kinect_marker;
	Vector3d t_kinect_marker( x_cor,y_cor,z_cor);
	T_kinect_marker.block(0,3,3,1) = t_kinect_marker;
	T_kinect_marker.row(3) << 0,0,0,1;


	MatrixXd T_kinect_arm =  T_kinect_marker*T_marker_arm2;
	//cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	if(minimumid ==300)
	{
	pub1.publish(array);
	transformation_init = true;
	}
	//pub1.publish(array);//side
	
	
}
break;

	
case 350:
	
	{//if(msg->markers[i].id == 322){

MatrixXd T_marker_arm3(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;


    R <<  0, 0, -1,
           0, -1, 0,
          1, 0, 0;
          


T_marker_arm3.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
t_marker_arm << 0.05,-0.075,-1.075;
T_marker_arm3.block(0,3,3,1) = t_marker_arm;
T_marker_arm3.row(3) << 0,0,0,1;

	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	x=msg2.Multiplemarkers[i].pose.orientation.x;
	y=msg2.Multiplemarkers[i].pose.orientation.y;
	z=msg2.Multiplemarkers[i].pose.orientation.z;
	w=msg2.Multiplemarkers[i].pose.orientation.w;
	
	
	Quaterniond q(x, y, z, w);
	q.normalize();
	R1 = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 

	MatrixXd T_kinect_marker(4,4); 
	Matrix3d R_kinect_marker;

	R_kinect_marker << R1(0),R1(1),R1(2),
	      R1(3),R1(4),R1(5),
	      R1(6),R1(7),R1(8);
	     
	T_kinect_marker.block(0,0,3,3) = R_kinect_marker;
	Vector3d t_kinect_marker( x_cor,y_cor,z_cor);
	T_kinect_marker.block(0,3,3,1) = t_kinect_marker;
	T_kinect_marker.row(3) << 0,0,0,1;


	MatrixXd T_kinect_arm =T_kinect_marker*T_marker_arm3;
	//cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	
	
	if(minimumid ==350)
	{
	pub1.publish(array);
	transformation_init = true;
	}
	//pub1.publish(array);//top marker
	
	
	}
break;
case 400:
{	//if(msg->markers[i].id == 322){

MatrixXd T_marker_arm4(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;


    R <<  -1, 0, 0,
          0, 1, 0,
          0, 0, -1;

     

T_marker_arm4.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
t_marker_arm << -1.175,-0.05,0.075;
T_marker_arm4.block(0,3,3,1) = t_marker_arm;
T_marker_arm4.row(3) << 0,0,0,1;

	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	x=msg2.Multiplemarkers[i].pose.orientation.x;
	y=msg2.Multiplemarkers[i].pose.orientation.y;
	z=msg2.Multiplemarkers[i].pose.orientation.z;
	w=msg2.Multiplemarkers[i].pose.orientation.w;
	
	
	Quaterniond q(x, y, z, w);
	q.normalize();
	R1 = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 

	MatrixXd T_kinect_marker(4,4); 
	Matrix3d R_kinect_marker;

	R_kinect_marker << R1(0),R1(1),R1(2),
	      R1(3),R1(4),R1(5),
	      R1(6),R1(7),R1(8);
	     
	T_kinect_marker.block(0,0,3,3) = R_kinect_marker;
	Vector3d t_kinect_marker( x_cor,y_cor,z_cor);
	T_kinect_marker.block(0,3,3,1) = t_kinect_marker;
	T_kinect_marker.row(3) << 0,0,0,1;


	MatrixXd T_kinect_arm =T_kinect_marker*T_marker_arm4;
	//cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	if(minimumid ==400)
	{
	pub1.publish(array);
	transformation_init = true;
	}
	//pub1.publish(array);
	

	
	}
break;
	
case 250:
{	//if(msg->markers[i].id == 322){

MatrixXd T_marker_arm5(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;

/*
    R <<  0, -1, 0,
          1, 0, 0,
          0, 0, -1;
          
  R <<    -1, 0, 0,
          0, -1, 0,
          0, 0, 1;*/
          
          
   R <<    0, 1, 0,
          -1, 0, 0,
           0, 0, 1;


T_marker_arm5.block(0,0,3,3) = R.inverse();
Vector3d t_marker_arm;
//t_marker_arm << -1.075,-0.05,-0.075;
t_marker_arm << 0.05,1.075,0.075;
T_marker_arm5.block(0,3,3,1) = t_marker_arm;
T_marker_arm5.row(3) << 0,0,0,1;

	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	x=msg2.Multiplemarkers[i].pose.orientation.x;
	y=msg2.Multiplemarkers[i].pose.orientation.y;
	z=msg2.Multiplemarkers[i].pose.orientation.z;
	w=msg2.Multiplemarkers[i].pose.orientation.w;
	
	
	Quaterniond q(x, y, z, w);
	q.normalize();
	R1 = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 

	MatrixXd T_kinect_marker(4,4); 
	Matrix3d R_kinect_marker;

	R_kinect_marker << R1(0),R1(1),R1(2),
	      R1(3),R1(4),R1(5),
	      R1(6),R1(7),R1(8);
	     
	T_kinect_marker.block(0,0,3,3) = R_kinect_marker;
	Vector3d t_kinect_marker( x_cor,y_cor,z_cor);
	T_kinect_marker.block(0,3,3,1) = t_kinect_marker;
	T_kinect_marker.row(3) << 0,0,0,1;


	MatrixXd T_kinect_arm =  T_kinect_marker*T_marker_arm5;
	//cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	
	if(minimumid ==250)
	{
	pub1.publish(array);
	transformation_init = true;
	}
	//pub1.publish(array);
	
	
	}
break;
	
	
	}
	
	if(transformation_init){
	Matrix3d R2;
	
	msg3.header.stamp = ros::Time::now();
	msg3.header.frame_id = "arm_frame";
	
	x_cor=msg2.Multiplemarkers[i].pose.position.x;
	y_cor=msg2.Multiplemarkers[i].pose.position.y;
	z_cor=msg2.Multiplemarkers[i].pose.position.z;


	
	Quaterniond q2(msg->markers[i].pose.pose.orientation.x, msg->markers[i].pose.pose.orientation.y, msg->markers[i].pose.pose.orientation.z, msg->markers[i].pose.pose.orientation.w);
	q2.normalize();
	//cout<<"q2"<<q2;
	R2 = q2.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix 
	//cout<<"R2"<<R2;
	MatrixXd T_kinect_marker2(4,4); 
	Matrix3d R_kinect_marker2;

	R_kinect_marker2 << R2(0),R2(1),R2(2),
	      		   R2(3),R2(4),R2(5),
	      		   R2(6),R2(7),R2(8);
	     
	T_kinect_marker2.block(0,0,3,3) = R_kinect_marker2;
	
	Vector3d t_kinect_marker2( x_cor,y_cor,z_cor);
	T_kinect_marker2.block(0,3,3,1) = t_kinect_marker2;
	T_kinect_marker2.row(3) << 0,0,0,1;
	
	MatrixXd T_marker_arm2(4,4);
	
	
	
	T_marker_arm2 =  T_kinect_arm*T_kinect_marker2 ;
	//cout<<"T_marker_arm2\n\n\n"<<T_marker_arm2;
	
	msg3.Multiplemarkers[i].id = msg->markers[i].id;
	
	//msg3.Multiplemarkers[i].pose.position.x = T_marker_arm2(4);
	//msg3.Multiplemarkers[i].pose.position.y = T_marker_arm2(8);
	//msg3.Multiplemarkers[i].pose.position.z = T_marker_arm2(12);
	
	msg3.Multiplemarkers[i].pose.position.x = T_marker_arm2(0,3);
	msg3.Multiplemarkers[i].pose.position.y = T_marker_arm2(1,3);
	msg3.Multiplemarkers[i].pose.position.z = T_marker_arm2(2,3);
	Matrix3d R3;
	R3 = T_marker_arm2.block(0,0,3,3);
	Quaterniond q3(R3);
	msg3.Multiplemarkers[i].pose.orientation.x = q2.x();
	msg3.Multiplemarkers[i].pose.orientation.y = q2.y();
	msg3.Multiplemarkers[i].pose.orientation.z = q2.z();
	msg3.Multiplemarkers[i].pose.orientation.w = q2.w();
	
	
	
	}


}

//cout<<"Minimum distance\n"<<minimum<<endl;
//usleep(0.01*1000000);
pub.publish(msg2);
pub2.publish(msg3);
cout<<"Minimum distance id \n"<<minimumid<<endl;

}






int main(int argc, char **argv) {


ros::init(argc, argv, "MultiplePoses");
nh = new ros::NodeHandle;
//new_msgs::MultipleMarkerPose multiple_poses;

ros::Rate loop_rate(100);

ros::Subscriber sub_1 = nh->subscribe ("/aruco_marker_publisher/markers",1,randomcall_back);
//ros::Subscriber sub_2 = nh.subscribe ("/aruco_marker_publisher/markers",1,randomcall_back1);
//ros::Subscriber sub_3 = nh.subscribe ("/kinect_armtransform",1,randomcall_back2);

//pub1 = nh->advertise<std_msgs::Float64MultiArray>("/kinect_armtransform", 1);

//pub.publish(multiple_poses);

while (ros::ok())
  {
  /*
MatrixXd T_kinect_arm(4,4); 
Matrix3d R_marker_arm;
Matrix3d R,R1,R2;
R << 1,0,0,
     0,1,0,
     0,0,1;
     
     R1 << 0,-1,0,
          0,0,-1,
          1,0,0;
     
R2 = R1.inverse();
T_kinect_arm.block(0,0,3,3) = R2;
Vector3d t_marker_arm;
//t_marker_arm << -0.025,-1.075,-0.05;
t_marker_arm << -0.075,-1.075,-0.05;
T_kinect_arm.block(0,3,3,1) = t_marker_arm;
T_kinect_arm.row(3) << 0,0,0,1;

//	T_kinect_arm = T_kinect_marker*T_marker_arm;
cout << T_kinect_arm << "\n\n";
	array.data.resize(16);
	array.data[0]=T_kinect_arm(0,0);
	array.data[1]=T_kinect_arm(0,1);
	array.data[2]=T_kinect_arm(0,2);
	array.data[3]=T_kinect_arm(0,3);
	array.data[4]=T_kinect_arm(1,0);
	array.data[5]=T_kinect_arm(1,1);
	array.data[6]=T_kinect_arm(1,2);
	array.data[7]=T_kinect_arm(1,3);
	array.data[8]=T_kinect_arm(2,0);
	array.data[9]=T_kinect_arm(2,1);
	array.data[10]=T_kinect_arm(2,2);
	array.data[11]=T_kinect_arm(2,3);
	array.data[12]=T_kinect_arm(3,0);
	array.data[13]=T_kinect_arm(3,1);
	array.data[14]=T_kinect_arm(3,2);
	array.data[15]=T_kinect_arm(3,3);
	
	pub1.publish(array);

loop_rate.sleep();*/
ros::spin();
//loop_rate.sleep();

}

return 0;



}

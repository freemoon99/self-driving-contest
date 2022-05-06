#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 96

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoint_X_Tor 0.1
#define WayPoint_Y_Tor 0.3

double pos_x = 0.0;
double pos_y = 0.0;
double roll,pitch,yaw;
int steer_angle     = STEER_NEUTRAL_ANGLE;

struct Point 
{ 
	float x; 
	float y; 
	float z;
};

struct WayPoints
{
	double x;
	double y;	
} ;

geometry_msgs::Pose2D my_pose,my_target_pose_goal;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}

void targetPoseCallback(const geometry_msgs::Pose2D& msg)
{
	my_target_pose_goal.x = msg.x;
	my_target_pose_goal.y = msg.y;
	my_target_pose_goal.theta = msg.theta;
}

int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "waypoint_race_car_control");

  ros::NodeHandle n;
  
  ros::Subscriber sub1 = n.subscribe("/slam_out_pose",10, &poseCallback);
  ros::Subscriber sub2 = n.subscribe("/pose_goal",10, &targetPoseCallback);
   
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/W_SteerAngle_Int16", 10);
  //ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
  
  ros::Rate loop_rate(20);  // 10
  
  int count = 0;
  int waypoint_arrival_flag = 0 ;
  double yaw_angle_current, yaw_angle_target,angle_rad;
  double waypoint_pos_base_link_x     = 0.0;
  double waypoint_pos_base_link_y     = 0.0; 
  double waypoint_pos_base_link_theta = 0.0; 
  double error, error_old, error_d, error_sum;
  double pi_gain,pd_gain,p_gain;
  
  
  error = error_old = error_d = error_sum = 0.0;
  p_gain = 0.9;
  pd_gain = 3; 
  
  
  while (ros::ok())
  {
	double tf_base_map_x,tf_base_map_y; 
	
	/*
	my_target_pose_goal.x =  3.840393;
	my_target_pose_goal.y = -7.042702;
    my_pose.x =  5.200000;
    my_pose.y = -7.200000;
	my_pose.theta = DEG2RAD(-180);	
	*/
	printf("\n\n\n\n\nmy pose : %lf %lf %lf\n", my_pose.x,my_pose.y,RAD2DEG(my_pose.theta));
	printf("goal pose : %lf %lf \n", my_target_pose_goal.x,my_target_pose_goal.y);	
	   
    tf_base_map_x = - my_pose.x;   //ㅅㅏㅇㄷㅐ ㅈㅏㅗㅍㅛㄹㅗ ㅂㅕㄴㅎㅗㅏㄴ  no translation
	tf_base_map_y = - my_pose.y;
	
		
	
	tf_base_map_x += my_target_pose_goal.x;   //ㅅㅏㅇㄷㅐ ㅈㅏㅗㅍㅛㄹㅗ ㅂㅕㄴㅎㅗㅏㄴ  no translation
	tf_base_map_y += my_target_pose_goal.y;     
    
    printf("TF base_link to map : %lf %lf\n", tf_base_map_x,tf_base_map_y);	
    
	waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
	waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);
	
    printf("Way point(base_link) : %lf %lf\n", waypoint_pos_base_link_x,waypoint_pos_base_link_y);	
	
	double angle_temp = RAD2DEG(atan2f(waypoint_pos_base_link_y,waypoint_pos_base_link_x));
	
	printf("angle temp %6.3lf \n",angle_temp);
	
	/*
	if(waypoint_pos_base_link_x != 0) 	angle_temp = waypoint_pos_base_link_y / waypoint_pos_base_link_x;
	else angle_temp = 0;
	
    angle_temp =  RAD2DEG(angle_temp);
    angle_temp =  angle_temp-(int)(angle_temp/360)*360;
	*/
	
	if((angle_temp >=180) && (angle_temp<=360)) angle_temp = 360-angle_temp;
			
	printf("steering angle temp %6.3lf \n",angle_temp);
	
	  
	error = angle_temp ;
	error_d = error - error_old;
	
	steer_angle = (int)( p_gain * error + pd_gain * error_d  + 0.5   );
	
	printf("%d \n", steer_angle);
		
	std_msgs::Int16 s_angle;
	s_angle.data =  -steer_angle%360;
	
	if(waypoint_arrival_flag ==0)car_control_pub1.publish(s_angle);
	
	ROS_INFO("%d",s_angle.data);

	
	if(( waypoint_pos_base_link_x<= WayPoint_X_Tor)&&( waypoint_pos_base_link_x <= WayPoint_Y_Tor) )
    { 
	   ROS_INFO("Arrvied at My WayPoint !"); 
	   waypoint_arrival_flag  = 1;	   
	} 
	else
	{
       waypoint_arrival_flag = 0;
	}
	
	error_old = error;
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

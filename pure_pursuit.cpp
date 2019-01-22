#include <ros/ros.h>
#include<iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include "prius_msgs/Control.h"
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>
#include <vector>
#include <math.h>

using namespace std;

float max_vel = 6.0; // maximum linear velocity
float steer;
float k = 0.8; // constant for relating look ahead distance and velocity
float wheelbase = 1.983; // wheel base for the vehicle
float d_lookahead = 0.08; // look ahead distance to calculate target point on path
float ep;
float cp;
float x_bot;
float y_bot;
float yaw;
float vel;
// nav_msgs::Odometry odom_sub;
// nav_msgs::Path path_sub;


geometry_msgs::Twist cmd,cmd1;
// cout <<"start"<< endl;
typedef struct Point 
{
	float x;
	float y;
} Point ;

float dist(geometry_msgs::PoseStamped a,float x,float y)
{	
	// Calculates distance between two points.
	// :param a [float]
	// :param x [float]
	// :param y [float]
	//calculate distance
	return sqrt(pow((a.pose.position.x - x),2) + pow((a.pose.position.y - y),2));
}
float pure_pursuit(Point goal_point)
{
	// Calculates the steering angle required for path tracking
	// :params goal_point [float,float] goal point coordinates
	// :params Delta [float] steering angle in radians 
	float tx = goal_point.x;
	float ty = goal_point.y;
	cout<< "yaw:"<< yaw <<endl;
	// measuring the slope of path point
	cout <<"slope:"<< atan2(ty - y_bot, tx - x_bot) << endl;
	// measuring heading angle 
	float alpha = atan2(ty - y_bot, tx - x_bot) - yaw ;
	cout <<"alpha:"<< alpha << endl;
	float Lf = k * max_vel + d_lookahead;
	// measuring the steering angle using pure pursuit controller
	float Delta = atan2(2.0 * wheelbase * sin(alpha) / Lf, 1);
	cout <<"Delta:"<<Delta<< endl;
	return Delta;
}
void callback_path(const nav_msgs::Path::ConstPtr& data)
{	
	// calculates target path point and the steering angle
	// :param data [Path]
	// :param ep [float]
	// calculate minimum distance 
	cout << "entered path callback " << endl;
	cout << (data->poses).size() << endl;
	float max_index= (data->poses).size();
	cout << "max_index is: " << max_index << endl;
	vector <float> distances;
	for(int i=0;i<max_index;i++)
	{
		distances.push_back( dist((data->poses[i]),x_bot,y_bot));
	}

	ep= *min_element(distances.begin(), distances.end()); //min element value
	cp= min_element(distances.begin(),distances.end()) - distances.begin(); //index of min element

	cout<<"old index:"<< cp<<endl;
	// calculate index of target point on path
	prius_msgs::Control prius_vel;
	float L=0;
	float Lf = k * max_vel + d_lookahead;
	float dx,dy;
	while (Lf > L && (cp + 1) < max_index)
	{
		dx = data->poses[cp + 1].pose.position.x - data->poses[cp].pose.position.x;
		dy = data->poses[cp + 1].pose.position.y - data->poses[cp].pose.position.y;
		L += sqrt(dx*dx + dy*dy);
		cp ++;
	}

	cout << max_index << endl;
	cout <<"new index is:"<< cp<< endl;

	Point goal_point;
	goal_point.x =data->poses[cp].pose.position.x;
	goal_point.y =data->poses[cp].pose.position.y;

	cout<<"current goal is:"<< goal_point.x<<" "<<goal_point.y<<endl;

	Point error;
	error.x = goal_point.x - x_bot;
	error.y = goal_point.y - y_bot;

	//cout<<error<<endl;
	float steer_angle = pure_pursuit(goal_point);

	cout<< "steer_angle :"<< steer_angle * 180 / M_PI;

	if(steer_angle * 180 / M_PI < -30) cmd.angular.z =-30;
	else if(steer_angle * 180 / M_PI > 30) cmd.angular.z =30;
	else cmd.angular.z =steer_angle * 180 / M_PI;

	cmd.linear.y = sqrt(error.x*error.x + error.y*error.y);
	cout<< "omega:"<< cmd.angular.z<<endl;
}
void callback_feedback(const nav_msgs::Odometry::ConstPtr& data)
{
	// Assigns the position of the robot to global variables from odometry.
	// :param x_bot [float]
	// :param y_bot [float]
	// :param yaw [float]
	// :param vel [float]
	x_bot = data->pose.pose.position.x;
	y_bot = data->pose.pose.position.y;
	
	// quarternion to euler conversion
	float siny = 2.0 * (data->pose.pose.orientation.w *
				   data->pose.pose.orientation.z +
				   data->pose.pose.orientation.x *
				   data->pose.pose.orientation.y);
	float cosy = 1.0 - 2.0 * (data->pose.pose.orientation.y *
						 data->pose.pose.orientation.y +
						 data->pose.pose.orientation.z *
						 data->pose.pose.orientation.z);

	yaw = atan2(siny, cosy) ;//yaw in radians

	cout<<"x of car:"<<x_bot<<endl;
	cout<<"y of car:"<< y_bot<<endl;
	cout<<"angle of car:"<<endl;
	cout<<"c"<<endl;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pure_pursuit");
	ros::NodeHandle nh;
	ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("cmd_delta", 10);
  	ros::Subscriber odom_sub = nh.subscribe("base_pose_ground_truth",10, callback_feedback);
  	ros::Subscriber path_sub = nh.subscribe("astroid_path",10, callback_path);

	ros::Rate r(10);
    while(ros::ok())
    {
        pub1.publish(cmd);
		cout<< "cmd published" <<endl;
        r.sleep();
        ros::spinOnce();
    }
    return 0;


}



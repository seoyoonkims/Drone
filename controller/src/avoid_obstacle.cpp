#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include <iostream>

#define RadToDeg 180/M_PI

geometry_msgs::PoseStamped current_position;
geometry_msgs::TwistStamped setpoint_vel;

float home_x = 0.0;
float home_y = 0.0;
float home_z = 1.0;

float cur_x = 0.0;
float cur_y = 0.0;
float cur_z = 0.0;

float q[4] = {0};
float turning_points[3] = {0, 0, 1};
float obstacle_points[3] = {0, 0, 1};
float cur_roll = 0;
float cur_pitch = 0;
float cur_yaw = 0;

float vel_limit = 0.3;
float yaw_limit = 0.15;

bool is_takeoff = false;
bool left_obstacle = false;
bool right_obstacle = false;

float obstacle_distance = 0.0;
float target_yaw = 0.0;
int mode = 1;

void QuaternionToEuler(float& roll, float& pitch, float& yaw);
void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void DistanceCallback(const std_msgs::Float32::ConstPtr& msg);
void Run();
void GoStraight(); // +x axis direction
void GoLeft(); // +y axis direction
void Stop();
void Turn(const float& target_yaw);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_node");
    ros::NodeHandle nh;
    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose", 10, PositionCallback)
    ros::Subscriber distance_sub = nh.subscribe("/distance", 10, DistanceCallback);
    ros::Publisher velocity_pub = nh.advertise <geometry_msgs::PoseStamped> ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Rate rate(20);

    float dist;

    while(ros::ok()) {
	    if(is_takeoff == false) 
        {
		    dist = sqrt((home_x - cur_x)^2 + (home_y - cur_y)^2 + (home_z - cur_z)^2);

            // take off
		    setpoint_vel.twist.linear.x = 0.5 * (home_x - cur_x);
    		setpoint_vel.twist.linear.y = 0.5 * (home_y - cur_y);
	    	setpoint_vel.twist.linear.z = 0.5 * (home_z - cur_z);
	    	velocity_pub.publish(setpoint_vel);
			
	    	if((dist < 0.3) && (q[2] > home_z)) {
                is_takeoff = true;
            }
	    }
	    else Run();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void Run()
{
    switch (mode)
    {
        case 0: // stop
            Stop();
            break;
        case 1: // go straight
            GoStraight();
            if (obstacle_distance < 0.3) { // 30cm
                Stop();
                obstacle_points[0] = cur_x;
                obstacle_points[1] = cur_y;
                obstacle_points[2] = cur_z;
                mode = 2;
            }
            break;
        case 2: // turn left
            Stop();
            target_yaw -= M_PI/2;
            Turn(target_yaw);
            if (abs(cur_yaw - target_yaw) < 0.5) // 0.5 radian 
            {
                if (obstacle_distance < 0.3) {
                    left_obstacle = true;
                    mode = 4;
                }
                else {
                    turning_points[0] = cur_x;
                    turning_points[1] = cur_y;
                    turning_points[2] = cur_z;
                
                    left_obstacle = false;
                    right_obstacle = false;
                    mode = 1;
                }
            }
            break;

        case 3: // turn back
            Stop();
            target_yaw += M_PI;
            Turn(target_yaw);
            if (abs(cur_yaw - target_yaw) < 0.5) // 0.5 radian
            {
                if (obstacle_distance < 0.3) {
                    mode = 4;
                }
                else {
                    turning_points[0] = cur_x;
                    turning_points[1] = cur_y;
                    turning_points[2] = cur_z;

                    left_obstacle = false;
                    right_obstacle = false;
                    mode = 1;
                }
            }
            break;

        case 4: // move left
            GoLeft();
            if (obstacle_distance > 0.3) {
                turning_points[0] = cur_x;
                turning_points[1] = cur_y;
                turning_points[2] = cur_z;
                left_obstacle = false;
                right_obstacle = false;
                mode = 1;
                break;
            }
            if (sqrt((cur_x - turning_points[0])^2 + (cur_x - turning_points[0])^2 + (cur_x - turning_points[0])^2) < 0.1)
            {
                Stop();
                if (left_obstacle == true) 
                {
                    right_obstacle == true;
                    mode = 0;
                }
                mode = 3;
            }
            break;

        default:
            break;
    }
    return;
}

void GoStraight()
{
    float vel_x = 0.5 * obstacle_distance;
    RegulateVelocity(vel_x);
    setpoint_vel.twist.linear.x = vel_x;
    setpoint_vel.twist.linear.y = 0;
    setpoint_vel.twist.linear.z = 0;
    local_vel_pub.publish(setpoint_vel);  
}

void GoLeft()
{
    setpoint_vel.twist.linear.x = 0;
    setpoint_vel.twist.linear.y = 0.1;
    setpoint_vel.twist.linear.z = 0;
    local_vel_pub.publish(setpoint_vel);  
}


void Turn(const float& target_yaw) {
	float current_yaw = current_attitude[2];
	float yaw_diff = target_yaw - current_yaw;

    CorrectAngle(yaw_diff)
	float cmd_r = yaw_diff * 3.0;
	RegulateVelocity(cmd_r, yaw_limit);
	
	setpoint_vel.twist.angular.z = cmd_r;
}

void Stop()
{
    setpoint_vel.twist.linear.x = 0;
    setpoint_vel.twist.linear.y = 0;
    setpoint_vel.twist.linear.z = 0;
    local_vel_pub.publish(setpoint_vel);  
}

void CorrectAngle(float& angle)
{
    while(abs(angle) > M_PI) 
    {
        if (angle > 0)
            angle -= 2 * M_PI;
        else
            angle += 2 * M_PI;
    }
}

void RegulateVelocity(float& vel, const float limit)
{
	if(abs(vel) > limit) {
		vel = vel / abs(vel) * limit;	
	}
}

void QuaternionToEuler(float& roll, float& pitch, float& yaw)
{
	// roll (x-axis rotation)
    float t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    float t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	float t2 = +2.0 * (q[3] * q[1] + q[2] * q[0]);
	t2 = t2 > 1.0 ? 1.0 :t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
    float t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    float t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2]*q[2]);
	yaw = std::atan2(t3, t4);
}

void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;

    q[0] = current_position.pose.orientation.x;
	q[1] = current_position.pose.orientation.y;
	q[2] = current_position.pose.orientation.z;
	q[3] = current_position.pose.orientation.w;

    cur_x = current_position.pose.position.x;
    cur_y = current_position.pose.position.y;
    cur_z = current_position.pose.position.z;

	QuaternionToEuler(cur_roll, cur_pitch, cur_yaw);
}

void DistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
   obstacle_distance = msg->data; // unit: meter
}

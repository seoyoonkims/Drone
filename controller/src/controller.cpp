#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#define RadToDeg 180/M_PI

geometry_msgs::PoseStamped cur_pose;        // position and orientation
geometry_msgs::TwistStamped setpoint_vel;   // linear and angular velocity
mavros_msgs::State cur_state;               // connection status, arming status, mode
mavros_msgs::SetMode land_set_mode;         // "AUTO", "GUIDED", "RTL"

std::vector<float> data; // dynamic sized array

ros::Publisher local_vel_pub;

// requests to a ROS service provided by another node in the network 
ros::ServiceClient take_picture_client;
ros::ServiceClient set_mode_client;

float home_x = 0.0;
float home_y = 0.0;
float home_z = 1.0;

float cur_image_x = 0.5;
float cur_image_y = 0.5;
float human_length = 0.3;

double vel_x, vel_y, vel_z;
double ang_yaw = 0;
double q[4];                    // quaternion
double current_attitude[3];     // Euler
double vel_limit = 0.3;
double yaw_limit = 0.15;
double target_yaw = 0;

bool mission_complete = true;
bool is_home = false;
bool is_takeoff = false;
bool pic_taken = false;

int star_idx = 0;
int mission_mode = 0;

// 0 : default
// 1 : move to home (cross hand)
// 2 : linearly follow target (left hand)
// 3 : angularly follow target (right hand)
// 4 : selfie (both hand)
// 5 : draw star (left leg)
// 6 : landing (right leg)


/* Convert quaternion to Euler. */
void QuaternionToEuler(double& roll, double& pitch, double& yaw)
{
    // roll
    double t0 = 2.0 * (q[3] * q[0] + q[1] * q[2]);
    double t1 = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    roll = std::atan2(t0, t1);

    // pitch
    double t2 = 2.0 * (q[3] * q[1] + q[2] * q[0]);
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    pitch = std::asin(t2);

    // yaw
    double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2]*q[2]);
	yaw = std::atan2(t3, t4);
}

/* Get current position /mavros/local_position/pose */
void cur_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cur_pose = *msg;

    q[0] = cur_pose.pose.orientation.x;
    q[1] = cur_pose.pose.orientation.y;
    q[2] = cur_pose.pose.orientation.z;
    q[3] = cur_pose.pose.orientation.w;

    // save results to current_attitude
    QuaternionToEuler(current_attitude[0], current_attitude[1], current_attitude[2]);
}

/* Get current state /mavros/state */
void mavros_State_Callback(const mavros_msgs::State::ConstPtr& msg) {
    cur_state = *msg;
}

/* Get current gesture /posenet/gesture_out */
void gesture_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    data = msg->data;
}

void RegulateVelocity(double& vel, const double limit)
{
    if (abs(vel) > limit) {
        // to preserve direction
        vel = vel / abs(vel) * limit;
    }
}

void ControlYaw()
{
    double current_yaw = current_attitude[2];
    double yaw_diff = target_yaw - current_yaw;

    // arrange to [-pi, pi]
    while(abs(yaw_diff) > M_PI) {
        if(yaw_diff > 0) {
            yaw_diff = yaw_diff - 2 * M_PI; 
        }
        else {
            yaw_diff = yaw_diff + 2 * M_PI;
        }
    }

    double cmd_ang_vel = yaw_diff * 3.0; // angular velocity
    RegulateVelocity(cmd_ang_vel, yaw_limit); // check if it exceeds limit velocity

    setpoint_vel.twist.angular.z = cmd_ang_vel;
}

void run()
{
    int num_person = data.size() / 4;
    float cur_x, cur_y, cur_z;

    ControlYaw();

    // detecting mission
    for(int i = 0; i < num_person; i++) {
        // data is float type
        if(data[4*i] == 1.0) // cross hand (move to home)
        {
            if(mission_complete){
                mission_mode = 1;
                mission_complete = false;
            }
            break;
        }
        else if(data[4*i] == 2.0) // left hand (linearly follow target)
        {
            cur_image_x = data[4*i+1];
            cur_image_y = data[4*i+2];
            human_length = data[4*i+3];

            if (mission_complete){
                mission_mode = 2;
                mission_complete = false;
            }
            break;
        }
        else if (data[4*i] == 3.0) // right hand (angularly follow target)
        {
            cur_image_x = data[4*i+1];
            cur_image_y = data[4*i+2];
            human_length = data[4*i+3];

            if (mission_complete){
                mission_mode = 3;
                mission_complete = false;
                pic_taken = false;
            }
            break;
        }
        else if (data[4*i] == 4.0) // both hand (selfie)
        {
            if (mission_complete){
                mission_mode = 5;
                mission_complete = false;
            }
            break;
        }
        else if (data[4*i] == 5.0) // left leg (draw star)
        {
            if (mission_complete){
                mission_mode = 4;
                mission_complete = false;
            }
            break;
        }
        else if (data[4*i] == 6.0) // right leg (landing)
        {
            if (mission_complete){
                mission_mode = 5;
                mission_complete = false;
            }
            break;
        }
        else continue; // continue until other gesture is detected
    }

    // conducting mission
    if (mission_mode == 0) // default
    {
        setpoint_vel.twist.linear.x = 0;
	    setpoint_vel.twist.linear.y = 0;
	    setpoint_vel.twist.linear.z = 0;
	    local_vel_pub.publish(setpoint_vel);
    }
    else if (mission_mode == 1) // cross hand (move to home)
    {
        // arrange yaw first
        if (std::abs(current_attitude[2]) > M_PI / 60) // 단위 확인 필요
        {
            target_yaw = 0;
            ControlYaw();
            continue;
        }

        home_x = 0.0;
        home_y = 0.0;
        home_z = 1.0;

        cur_x = cur_pose.pose.position.x;
        cur_y = cur_pose.pose.position.y;
        cur_z = cur_pose.pose.position.z;

        if ((std::abs(home_x - cur_x) < 0.2) && (std::abs(home_y - cur_y) < 0.2) && (std::abs(home_z - cur_z) < 0.2)){
            mission_complete = true;
            mission_mode = 0;

            setpoint_vel.twist.linear.x = 0;
            setpoint_vel.twist.linear.y = 0;
            setpoint_vel.twist.linear.z = 0;
            local_vel_pub.publish(setpoint_vel);
        }
        else {
            vel_x = 0.5 * (home_x - cur_x);
            vel_y = 0.5 * (home_y - cur_y);
            vel_z = 0.5 * (home_z - cur_z);

			RegulateVelocity(vel_x,vel_limit);
			RegulateVelocity(vel_y,vel_limit);
			RegulateVelocity(vel_z,vel_limit);

            ROS_INFO("mission mode 1.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", vel_x, vel_y, vel_z);
    
            setpoint_vel.twist.linear.x = vel_x;
            setpoint_vel.twist.linear.y = vel_y;
            setpoint_vel.twist.linear.z = vel_z;

            local_vel_pub.publish(setpoint_vel);
        }
    }
    else if (mission_mode == 2) // left hand (linearly follow target)
    {
        float target_x = 0.5;
        float target_y = 0.5;
        float required_length = 0.2;

        if ((std::abs(target_x - cur_image_x) < 0.1) && (std::abs(target_y - cur_image_y) < 0.1) && (std::abs(required_length - human_length) < 0.05)){
            mission_complete = true;
            mission_mode = 0;

            setpoint_vel.twist.linear.x = 0;
            setpoint_vel.twist.linear.y = 0;
            setpoint_vel.twist.linear.z = 0;
            local_vel_pub.publish(setpoint_vel);
        }
        else
        {
            vel_x = -7.5 * (human_length - required_length);
            vel_y = -3 * (cur_image_x - target_x);
            vel_z = -1.5 * (cur_image_y - target_y);
	    	
			RegulateVelocity(vel_x, vel_limit);
			RegulateVelocity(vel_y, vel_limit);
			RegulateVelocity(vel_z, vel_limit);

            ROS_INFO("mission mode 2.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", vel_x, vel_y, vel_z);

            setpoint_vel.twist.linear.x = vel_x;
            setpoint_vel.twist.linear.y = vel_y;
            setpoint_vel.twist.linear.z = vel_z;

            local_vel_pub.publish(setpoint_vel);
        }
    }
    else if (mission_mode == 3) // right hand (angularly follow target)
    {
        float target_x = 0.5;
        float target_y = 0.5;
        float required_length = 0.2;

        if ((std::abs(target_x - cur_image_x) < 0.1) && (std::abs(target_y - cur_image_y) < 0.1) && (std::abs(required_length - human_length) < 0.05))
        {
            mission_complete = true;
            mission_mode = 0;

            setpoint_vel.twist.linear.x = 0;
            setpoint_vel.twist.linear.y = 0;
            setpoint_vel.twist.linear.z = 0;
            local_vel_pub.publish(setpoint_vel);
        }
        else
        {
            vel_x = -3 * (human_length - required_length);
            vel_y = 0;
            vel_z = -1.5 * (cur_image_y - target_y);
            ang_yaw = -1 * (cur_image_x - target_x); // 단위 조정 필요

            RegulateVelocity(vel_x, vel_limit);
			RegulateVelocity(vel_y, vel_limit);
			RegulateVelocity(vel_z, vel_limit);
            RegulateVelocity(ang_yaw, yaw_limit);

            ROS_INFO("mission mode 2.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", vel_x, vel_y, vel_z);

            setpoint_vel.twist.linear.x = vel_x;
            setpoint_vel.twist.linear.y = vel_y;
            setpoint_vel.twist.linear.z = vel_z;
            setpoint_vel.twist.angular.z = ang_yaw;

            local_vel_pub.publish(setpoint_vel);
            target_yaw = current_attitude[2];
        }
    }
    else if (mission_mode == 4) // both hand (selfie)
    {
        setpoint_vel.twist.linear.x = 0;
	    setpoint_vel.twist.linear.y = 0;
	    setpoint_vel.twist.linear.z = 0;

	    local_vel_pub.publish(setpoint_vel);
	
        ROS_INFO("mission mode 3.");

	    if(pic_taken == false) {
            // save picture in node_posenet
            pic_taken = true;
            mission_complete = true;
            mission_mode = 0;
	    }
    }
    else if (mission_mode == 5) // left leg (draw star)
    {
        // arrange yaw first
        if (std::abs(current_attitude[2]) > M_PI / 60) // 단위 확인 필요
        {
            target_yaw = 0;
            ControlYaw();
            continue;
        }
        // return to home
        if (is_home != true) {
            home_x = 0.0;
            home_y = 0.0;
            home_z = 1.0;

            cur_x = cur_pose.pose.position.x;
            cur_y = cur_pose.pose.position.y;
            cur_z = cur_pose.pose.position.z; 

            vel_x = 0.5 * (home_x - cur_x);
            vel_y = 0.5 * (home_y - cur_y);
            vel_z = 0.5 * (home_z - cur_z);

			RegulateVelocity(vel_x, vel_limit);
			RegulateVelocity(vel_y, vel_limit);
			RegulateVelocity(vel_z, vel_limit);

            ROS_INFO("mission mode 4.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", vel_x, vel_y, vel_z);
            setpoint_vel.twist.linear.x = vel_x;
            setpoint_vel.twist.linear.y = vel_y;
            setpoint_vel.twist.linear.z = vel_z;

            local_vel_pub.publish(setpoint_vel);

            if (std::abs(home_x - cur_x) < 0.3 && std::abs(home_y - cur_y) < 0.3 && std::abs(home_z - cur_z) < 0.3)
            {
                is_home = true;
            }
        }
        else 
        {
            // draw star
            std::vector<std::pair<double, double>> star_points = 
            {{0.0, 0.0}, {1.309, 0.951}, {-0.309, 0.951}, {1.0, 0.0}, {0.5, 1.539}, {0.0, 0.0}};
                
            double scale_factor = 3.0;
            int star_length = star_points.size();

            if (star_idx >= star_length)
            {
                mission_complete = true;
                mission_mode = 0;
                star_idx = 0;

		        setpoint_vel.twist.linear.x = 0;
            	setpoint_vel.twist.linear.y = 0;
            	setpoint_vel.twist.linear.z = 0;
            	local_vel_pub.publish(setpoint_vel);
            }
            else
            {
                std::pair<double,double> waypoint = star_points[star_idx];
                // set a new target point
                double temp_target_y = waypoint.first * scale_factor + home_y;
                double temp_required_length = waypoint.second * scale_factor + home_z;

                // set velocity
                cur_x = cur_pose.pose.position.x;
                cur_y = cur_pose.pose.position.y;
                cur_z = cur_pose.pose.position.z; 

                vel_x = 0;
                vel_y = 4* (temp_target_y - cur_y);
                vel_z = 4* (temp_required_length - cur_z);

				RegulateVelocity(vel_x, vel_limit);
				RegulateVelocity(vel_y, vel_limit);
				RegulateVelocity(vel_z, vel_limit);

                ROS_INFO("drawing a ♥");
                ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", vel_x, vel_y, vel_z);
                
                setpoint_vel.twist.linear.x = vel_x;
                setpoint_vel.twist.linear.y = vel_y;
                setpoint_vel.twist.linear.z = vel_z;             
                local_vel_pub.publish(setpoint_vel);

                // check if the current position is close enough to the target position
                if (std::abs(temp_target_y - cur_y) < 0.1 && std::abs(temp_required_length - cur_z) < 0.1)
                {
                    star_idx++;
                }
            }
        }        
    }
    else if(mission_mode == 6) // right leg (landing)
	{
        home_x = 0.0;
        home_y = 0.0;
        home_z = 1.0;
		
		vel_x = 0.8 * (home_x - cur_pose.pose.position.x);
		vel_y = 0.8 * (home_y - cur_pose.pose.position.y);
		vel_z = 0.8 * (home_z - cur_pose.pose.position.z);
		
		double dist = sqrt(pow(home_x - cur_pose.pose.position.x, 2) + pow(home_y - cur_pose.pose.position.y, 2) + pow(home_z - cur_pose.pose.position.z, 2));

		if(dist > 0.3)
		{
			setpoint_vel.twist.linear.x = vel_x;
			setpoint_vel.twist.linear.y = vel_y;
			setpoint_vel.twist.linear.z = vel_z;

            local_vel_pub.publish(setpoint_vel);
		}		
 		else
		{
			set_mode_client.call(land_set_mode);
			land_set_mode.response.mode_sent;						
		}	
	}
    else
    {
        ROS_INFO("mission mode 0.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gesture_out_subscriber"); // initialize node
    
    ros::NodeHandle nh;
    ros::Subscriber gesture_sub = nh.subscribe("/posenet/gesture_out", 10, gesture_Callback);
    ros::Subscriber cur_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, cur_Pose_Callback);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, mavros_State_Callback);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped> ("mavros/setpoint_velocity/cmd_vel", 10);

    // ros::ServiceClient take_picture_client = nh.serviceClient<std_srvs::SetBool> ("service/take_picture");
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

    land_set_mode.request.custom_mode = "AUTO.LAND";

    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    while(ros::ok())
    {
        if(is_takeoff == false)
        {
            double dist = sqrt(pow(home_x - cur_pose.pose.position.x, 2) + pow(home_y - cur_pose.pose.position.y, 2) + pow(home_z - cur_pose.pose.position.z, 2));

		    setpoint_vel.twist.linear.x = 0.5 * (home_x - cur_pose.pose.position.x);
		    setpoint_vel.twist.linear.y = 0.5 * (home_y - cur_pose.pose.position.y);
		    setpoint_vel.twist.linear.z = 0.5 * (home_z - cur_pose.pose.position.z);
		    
            local_vel_pub.publish(setpoint_vel);
			
		    if(dist < 0.3 && cur_pose.pose.position.z > home_z)
		    {
		    	is_takeoff = true;
		    }
        }
        
        run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
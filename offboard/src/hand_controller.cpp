#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <ncurses.h>

const int ROLL_CHANNEL = 0;
const int PITCH_CHANNEL = 1;
const int THROTTLE_CHANNEL = 2;
const int YAW_CHANNEL = 3;

const int MIN_RC_VALUE = 1000;
const int MAX_RC_VALUE = 2000;
const int MID_RC_VALUE = (MAX_RC_VALUE + MIN_RC_VALUE) / 2;

mavros_msgs::OverrideRCIn rc_msg;

void controlDrone(int roll, int pitch, int throttle, int yaw) {
    rc_msg.channels[ROLL_CHANNEL] = roll;
    rc_msg.channels[PITCH_CHANNEL] = pitch;
    rc_msg.channels[THROTTLE_CHANNEL] = throttle;
    rc_msg.channels[YAW_CHANNEL] = yaw;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_control_node");
    ros::NodeHandle nh;

    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    rc_msg.channels[0] = 1500;
    rc_msg.channels[1] = 1500;
    rc_msg.channels[2] = 1500;
    rc_msg.channels[3] = 1500;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    ros::Rate rate(15);

    while (ros::ok()) {
    
        int ch = getch();
        int roll_input = 0, pitch_input = 0, throttle_input = 0, yaw_input = 0;

        switch (ch) {
            case KEY_UP:
                pitch_input = MID_RC_VALUE + 100;
                break;
            case KEY_DOWN:
                pitch_input = MID_RC_VALUE - 100;
                break;
            case KEY_LEFT:
                roll_input = MID_RC_VALUE - 100;
                break;
            case KEY_RIGHT:
                roll_input = MID_RC_VALUE + 100;
                break;
            case 'w':
                throttle_input = MID_RC_VALUE + 100;
                break;
            case 's':
                throttle_input = MID_RC_VALUE - 100;
                break;
            case 'a':
                yaw_input = MID_RC_VALUE - 100;
                break;
            case 'd':
                yaw_input = MID_RC_VALUE + 100;
                break;
            default:
                break;
        }

        controlDrone(roll_input, pitch_input, throttle_input, yaw_input);
        rc_override_pub.publish(rc_msg);

        ros::spinOnce();
        rate.sleep();
    }
    endwin();

    return 0;
}

#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sensor_msgs/Joy.h>

ros::Publisher joy_pub;
sensor_msgs::Joy joy_msg;

int Ascii[] = {96, 49, 50, 51, 52, 53, 54, 55, 56, 57, 48, 68, 65, 67, 66}; //{`,1,2,3,4,5,6,7,8,9,0,←,↑,→,↓}

std::string msg = R"(

 ---------------------------
 buttons:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
 key borad:[`, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0]

 axes   :  [0, 0, 0, 0, 0, 0]
 key borad:[←, ↑, →, ↓]
 
 CTRL-C to quit
 
 )";

int Getch(void) // #include <termios.h> 참조
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

void JoyInit() // Joy 버튼 초기화
{
    joy_msg.buttons.clear();
    joy_msg.axes.clear();
    for (int i = 0; i < 11; i++)
    {
        joy_msg.buttons.push_back(0);
    }
    for (int i = 0; i < 6; i++)
    {
        joy_msg.axes.push_back(0.0);
    }
}

void JoyKeboard(int key_num) // 지정 값과 같은 값이 들어오면 버튼 수행
{
    JoyInit();
    for (int j = 0; j < 15; j++)
    {
        if (key_num == Ascii[j])
        {
            joy_msg.buttons.clear();
            joy_msg.axes.clear();

            if (j == 11)
            {
                joy_msg.axes.push_back(1.0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
            }
            else if (j == 13)
            {
                joy_msg.axes.push_back(-1.0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
            }
            else if (j == 12)
            {
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(1.0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
            }
            else if (j == 14)
            {
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(-1.0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
                joy_msg.axes.push_back(0);
            }
            else
            {
                for (int i = 0; i < 6; i++)
                {
                    joy_msg.axes.push_back(0);
                }
            }

            for (int i = 0; i < 11; i++)
            {
                if (i == j)
                {
                    joy_msg.buttons.push_back(1);
                    continue;
                }
                joy_msg.buttons.push_back(0);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_keyboard");
    ros::NodeHandle n;
    joy_pub = n.advertise<sensor_msgs::Joy>("/joy", 1);

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        std::cout << msg;
        int key = Getch();
        ROS_INFO_STREAM(key);
        if (key == 3) // ctrl+c
        {
            break;
        }
        if(key ==27 || key==91)
        {
            continue;
        }
        
        JoyKeboard(key);
        joy_pub.publish(joy_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
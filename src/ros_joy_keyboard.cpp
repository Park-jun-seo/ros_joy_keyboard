#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

ros::Publisher joy_pub;
sensor_msgs::Joy joy_msg;

int Ascii[] = {96, 49, 50, 51, 52, 53, 54, 55, 56, 57, 48, 68, 65, 67, 66}; //{`,1,2,3,4,5,6,7,8,9,0,←,↑,→,↓}
char key;

class KeyboardReader
{
public:
  KeyboardReader(): kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_iflag |= IGNBRK;
    raw.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    raw.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    // Setting a new line, then end of file
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  void readOne(char * c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }

  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};


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

KeyboardReader input;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_keyboard");
    ros::NodeHandle n;
    joy_pub = n.advertise<sensor_msgs::Joy>("/joy", 1);
    puts("---------------------------");
    puts("buttons:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]");
    puts("key borad:[`, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0]");
    puts("axes   :  [←→, ↑↓, 0, 0, 0, 0]");
    puts("key borad:[←, ↑, →, ↓]");
    puts("CTRL-C to quit");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        input.readOne(&key);
        //key = Getch();

        if (key == 3) // ctrl+c
        {
            break;
        }
        if (key == 27 || key == 91)
        {
            continue;
        }

        JoyKeboard(key);

        // ROS_INFO_STREAM(key);

        joy_pub.publish(joy_msg);
        key=0;
        tcflush(0, TCIFLUSH);
        ros::spinOnce();
        loop_rate.sleep();
    }
    input.shutdown();
    return 0;
}
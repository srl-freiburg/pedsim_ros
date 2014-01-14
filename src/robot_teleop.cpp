#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>
#include <termios.h>
#include <pedsim_msgs/AgentState.h>


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Teleop
{
public:
    Teleop();
    void keyLoop();

private:

    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher vel_pub_;
};

Teleop::Teleop():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<pedsim_msgs::AgentState>("robot_state", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


// main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_teleop");
    Teleop robot;

    signal(SIGINT,quit);

    robot.keyLoop();
  
    return(0);
}


// Key loop (for driving the robot)
void Teleop::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
    }
   
    double vang = a_scale_*angular_;
    double vlin = l_scale_*linear_;

    pedsim_msgs::AgentState astate;
    astate.id = 2;
    astate.velocity.x = vlin * cos(vang);
    astate.velocity.y = vlin * sin(vang);

    ROS_INFO("Linear, angular %f, %f", vlin, vang);

    if(dirty ==true)
    {
      vel_pub_.publish(astate);    
      dirty=false;
    }
  }


  return;
}

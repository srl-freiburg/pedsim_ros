#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>
#include <termios.h>
#include <pedsim_msgs/AgentState.h>
#include <std_msgs/Header.h>


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
    double rot_angle_;
    double robot_speed;
    ros::Publisher vel_pub_;
};

Teleop::Teleop():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0),
  rot_angle_(0.0),
  robot_speed(1.2)
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
  puts("Use arrow keys to move the robot.");


  // for(;;)
  while(true)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    // linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        rot_angle_ += 30.0;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        rot_angle_ -= 30.0;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        robot_speed += 0.1;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        robot_speed -= 0.1;
        break;
    }
   
    // double vang = a_scale_*angular_;
    // double vlin = l_scale_*linear_;
    // double baseline = 0.5;

    // double dx = (vlin + vang) / 2.0;
    // double dr = (vlin + vang) / baseline;

    // pedsim_msgs::AgentState astate;
    // astate.id = 1;
    // astate.velocity.x = vlin * cos(vang);
    // astate.velocity.y = vlin * sin(vang);

    // astate.velocity.x = (1.0 * dx) + (dr * baseline / 2.0);
    // astate.velocity.y = (1.0 * dx) - (dr * baseline / 2.0);

    ROS_INFO("Speed, Angle %f, %f", robot_speed, rot_angle_);



    /// using the code from old cpp project
    double angle = rot_angle_;
    double vx = cos(angle * M_PI/180.0);
    double vy = sin(angle * M_PI/180.0);

    double stepx = robot_speed * vx;
    double stepy = robot_speed * vy;

    pedsim_msgs::AgentState astate;

    std_msgs::Header header_;
    header_.stamp = ros::Time::now();
    astate.header = header_;

    // astate.id = 1;
    astate.type = 2;
    astate.velocity.x = stepx;
    astate.velocity.y = stepy;


    if(dirty ==true)
    {
      vel_pub_.publish(astate);    
      dirty=false;
    }
  }


  return;
}

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "JHPWMDriver/src/JHPWMPCA9685.h"
#include <math.h>
#define PI 3.141592654

const int THROTTLE_COMMAND_INDEX = 0;
const int STEERING_COMMAND_INDEX = 1;

const int THROTTLE_FORWARD_VALUE = 1000;
const int THROTTLE_STOPPED_VALUE = 0;
const int THROTTLE_REVERSE_VALUE = -1000;

const int THROTTLE_STOPPED_PWM = 380;
const int THROTTLE_FORWARD_PWM = 450;
const int THROTTLE_REVERSE_PWM = 330;
const int THROTTLE_REVERSE_MODE_PWM = 370;

PCA9685 *pca9685 = new PCA9685();

int map(int x, int in_min, int in_max, int out_min, int out_max);
int convert_throttle_value_to_PWM(int throttle_value);
void updateCarCommands(const nav_msgs::Odometry::ConstPtr &msg);
void delayMessage(const nav_msgs::Odometry::ConstPtr &msg);
double desireDelay = 0;

int main(int argc, char **argv)
{
  ROS_INFO("Start motor_controller node.");
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("car_commands", 1000, updateCarCommands);
  n.getParam("/motor_controller/desire_delay", desireDelay);

  int err = pca9685->openPCA9685();
  if (err < 0)
  {
    printf("Error: %d", pca9685->error);
    pca9685->closePCA9685();
    return 0;
  }

  pca9685->setAllPWM(0, 0);
  pca9685->reset();
  pca9685->setPWMFrequency(60);

  ros::spin();

  pca9685->closePCA9685();
  return 0;
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  int toReturn = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return toReturn;
}

int convert_throttle_value_to_PWM(float throttle_value)
{
  if (throttle_value < THROTTLE_REVERSE_VALUE)
  {
    return THROTTLE_REVERSE_PWM;
  }
  if (throttle_value > THROTTLE_FORWARD_VALUE)
  {
    return THROTTLE_FORWARD_PWM;
  }
  if (throttle_value < THROTTLE_STOPPED_VALUE)
  {
    return map(throttle_value, THROTTLE_REVERSE_VALUE, THROTTLE_STOPPED_VALUE, THROTTLE_REVERSE_PWM, THROTTLE_STOPPED_PWM);
  }
  if (throttle_value >= THROTTLE_STOPPED_VALUE)
  {
    return map(throttle_value, THROTTLE_STOPPED_VALUE, THROTTLE_FORWARD_VALUE, THROTTLE_STOPPED_PWM, THROTTLE_FORWARD_PWM);
  }
}

void updateCarCommands(const nav_msgs::Odometry::ConstPtr &msg)
{
  delayMessage(msg);
  float throttle_value = msg->pose.pose.position.x;
  int throttle_PWM_value = convert_throttle_value_to_PWM(throttle_value);
  pca9685->setPWM(THROTTLE_COMMAND_INDEX, 0, throttle_PWM_value);
  // ROS_INFO("throttle_value: %f, throttle_PWM_value: %d", throttle_value, throttle_PWM_value);
}

void delayMessage(const nav_msgs::Odometry::ConstPtr &msg)
{
  double delay = (ros::Time::now() - msg->header.stamp).toSec();
  ros::Rate loop_delay_rate(100);
  while (desireDelay > delay)
  {
    delay = (ros::Time::now() - msg->header.stamp).toSec();
    loop_delay_rate.sleep();
    // ROS_INFO("delay: %f", delay);
  }
}

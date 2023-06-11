#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "JHPWMDriver/src/JHPWMPCA9685.h"
#include <math.h>
#define PI 3.141592654

const int THROTTLE_COMMAND_INDEX = 0;
const int STEERING_COMMAND_INDEX = 1;

const int STEERING_RIGHT_VALUE = 0;
const int STEERING_LEFT_VALUE = 180;

const double deg2rad = PI / 180.0;
const double rad2deg = 1 / deg2rad;

// Steering servor max roation is between 60 deg to 120 deg
const int SERVO_MAX_ANGLE = 30;
const int SERVO_0_ANGLE_PWM = 178;
const int SERVO_90_ANGLE_PWM = 392;
const int SERVO_180_ANGLE_PWM = 606;

PCA9685 *pca9685 = new PCA9685();

int map(double x, int in_min, int in_max, int out_min, int out_max);
double convert_steering_angle_to_servo_angle(double steering_angle);
int convert_servo_angle_value_to_PWM(double angle);
void updateCarCommands(const nav_msgs::Odometry::ConstPtr &msg);
void delayMessage(const nav_msgs::Odometry::ConstPtr &msg);
double desireDelay = 0;
ros::Publisher pub;
std_msgs::Float64 steering_angle_msg;

int main(int argc, char **argv)
{
  ROS_INFO("Start steering_controller node.");
  ros::init(argc, argv, "steering_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("car_commands", 1000, updateCarCommands);
  pub = n.advertise<std_msgs::Float64>("/steering_controller/steering_angle", 1000);
  n.getParam("/steering_controller/desire_delay", desireDelay);

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

int map(double x, int in_min, int in_max, int out_min, int out_max)
{
  double toReturn = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return round(toReturn);
}

double convert_steering_angle_to_servo_angle(double steering_angle)
{
  double c = (19.95*15.45) / (18.61*20.91);
  double servo_angle = rad2deg * asin(sin(steering_angle * deg2rad) / c);
  return servo_angle + 90; // The Servo motor zero angle is not in the middle.
}

int convert_servo_angle_value_to_PWM(double angle)
{
  if (angle < 90 - SERVO_MAX_ANGLE)
  {
    angle = 90 - SERVO_MAX_ANGLE;
  }
  if (angle > 90 + SERVO_MAX_ANGLE)
  {
    angle = 90 + SERVO_MAX_ANGLE;
  }
  return map(angle, 0, 180, SERVO_0_ANGLE_PWM, SERVO_180_ANGLE_PWM);
}

void updateCarCommands(const nav_msgs::Odometry::ConstPtr &msg)
{
  delayMessage(msg);
  double steering_angle = msg->pose.pose.position.y;
  double servo_angle = convert_steering_angle_to_servo_angle(steering_angle);
  int steering_PWM_value = convert_servo_angle_value_to_PWM(servo_angle);
  pca9685->setPWM(STEERING_COMMAND_INDEX, 0, steering_PWM_value);
  
  // Publish the steering angle that executed.
  steering_angle_msg.data = steering_angle;
  pub.publish(steering_angle_msg);
  ROS_INFO("steering_angle: %f, steering_PWM_value: %d, servo_angle: %f", steering_angle, steering_PWM_value, servo_angle);
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

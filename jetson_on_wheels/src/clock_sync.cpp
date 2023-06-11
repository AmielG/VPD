#include "ros/ros.h"
#include "ros/time.h"
#include "jetson_on_wheels/ClockSync.h"
#include <math.h>

bool sync(jetson_on_wheels::ClockSync::Request &req, jetson_on_wheels::ClockSync::Response &res);

int main(int argc, char **argv)
{
  ROS_INFO("Start clock_sync node.");
  ros::init(argc, argv, "clock_sync");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/sync", sync);
  ros::spin();

  return 0;
}

bool sync(jetson_on_wheels::ClockSync::Request &req, jetson_on_wheels::ClockSync::Response &res)
{
  double now = ros::Time::now().toSec();
  double delay = now - req.previous_clock;
  double sync_error = now - req.predicted_clock - delay * 0.5;

  ROS_INFO("previos_clock: %f, predicted_clock: %f", req.previous_clock, req.predicted_clock);

  // Reset the delay and sync_error value when the sync process is started.
  if (req.previous_clock == 0){
    delay = 0;
    sync_error = 0;
  }

  res.now = now;
  res.delay = delay;
  res.sync_error = sync_error;

  ROS_INFO("now: %f, delay: %f, sync_error: %f", now, delay, sync_error);
  return true;
}

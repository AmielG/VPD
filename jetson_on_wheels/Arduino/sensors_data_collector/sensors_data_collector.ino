#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <math.h>

const int hallPin = 3;  // hall effect sensor out pin
int hallState = 0;
double velocity = 0; // [m] / [s]
int numberOfSamples = 5;
double * velocitySamples = (double*) calloc(numberOfSamples, sizeof(double));
double movingAvg = 0;
unsigned long StartTime = 0;
unsigned long timeDelta = 0;
double rad2deg = 180 / PI;
double kilometrage = 0;
double wheelRadius = 78 * 0.5 / 1000.0; // [m]
void resetKilometrage( const std_msgs::Bool& reset_msg);
double pushSample(double newSample);

ros::NodeHandle node_handle;
std_msgs::Float32MultiArray sensor_msg;
std_msgs::Bool reset_msg;
ros::Publisher speedometer_publisher("/sensors_data_collector/sensors_data", &sensor_msg);
ros::Subscriber<std_msgs::Bool> reset_kilometrage_sub("/sensors_data_collector/reset_kilometrage", &resetKilometrage );

void resetKilometrage( const std_msgs::Bool& reset_msg){
  if(reset_msg.data == 1){
    kilometrage = 0;
    sensor_msg.data[1] = 0;
  }
}

  
void setup() {  
  Serial.begin(9600);
  pinMode(hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin),hallSensorChanged,CHANGE); 
  node_handle.initNode();
  node_handle.advertise(speedometer_publisher);
  node_handle.subscribe(reset_kilometrage_sub);

  sensor_msg.data = (float*)malloc(sizeof(float) * 2);
  sensor_msg.data_length = 2;
}

void loop(){
  if(millis() - StartTime > 300){
    sensor_msg.data[1] = 0;
    sensor_msg.data[0] = 0;
    double oldestSample = pushSample(0);
    movingAvg = movingAvg + (0 - oldestSample) / (double)numberOfSamples;
  }
  speedometer_publisher.publish(&sensor_msg);
  node_handle.spinOnce();  
  delay(20);
}

void hallSensorChanged(){
  timeDelta = millis() - StartTime;
  velocity = ((PI * 0.5) / ((double)timeDelta / 1000)) * wheelRadius;
  double oldestSample = pushSample(velocity);
  kilometrage += PI * 0.5 * wheelRadius;
  movingAvg = movingAvg + (velocity - oldestSample) / (double)numberOfSamples;
  if (isnan(movingAvg)){
    movingAvg = 0;   
  }
  sensor_msg.data[0] = movingAvg;
  sensor_msg.data[1] = velocity;
  StartTime = millis();
}


double pushSample(double newSample)
{
  double oldestSample = velocitySamples[numberOfSamples - 1];
  for (int i = numberOfSamples - 1; i > 0; i--)
  {
      velocitySamples[i] = velocitySamples[i - 1];
  }
  velocitySamples[0] = newSample;
  return oldestSample;
}

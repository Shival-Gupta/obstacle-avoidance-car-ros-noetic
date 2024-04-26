#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String sensor_data;
ros::Publisher sensor_publisher("sensor_data", &sensor_data);

// Define pins for ultrasonic sensors
const int trigPin1 = 2;
const int echoPin1 = 3;
const int trigPin2 = 4;
const int echoPin2 = 5;
const int trigPin3 = 6;
const int echoPin3 = 7;

// Variables for storing distances
long distance1, distance2, distance3;

void setup()
{
  nh.initNode();
  nh.advertise(sensor_publisher);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop()
{
  // Read distance from sensor 1
  distance1 = getDistance(trigPin1, echoPin1);

  // Read distance from sensor 2
  distance2 = getDistance(trigPin2, echoPin2);

  // Read distance from sensor 3
  distance3 = getDistance(trigPin3, echoPin3);

  // Construct the string to publish
  String dataString = String(distance1) + " " + String(distance2) + " " + String(distance3);
  sensor_data.data = dataString.c_str();
  sensor_publisher.publish(&sensor_data);

  // Print distances over Serial
  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print(" cm, Distance 2: ");
  Serial.print(distance2);
  Serial.print(" cm, Distance 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  nh.spinOnce();
  delay(1000);
}

long getDistance(int trigPin, int echoPin)
{
  // Trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance
  long distance = duration * 0.034 / 2;

  return distance;
}
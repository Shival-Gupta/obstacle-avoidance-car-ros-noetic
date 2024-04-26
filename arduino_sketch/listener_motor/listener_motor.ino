#include <ros.h>
#include <std_msgs/Int16.h>
#include <AFMotor.h>

#define MAX_SPEED 190 // sets maximum speed of DC motors [max 255]

AF_DCMotor motorBL(1); // Back Left Motor (M1)
AF_DCMotor motorBR(2); // Back Right Motor (M2)
AF_DCMotor motorFR(3); // Front Right Motor (M3)
AF_DCMotor motorFL(4); // Front Left Motor (M4)

// Declare the motorCommandCallback function prototype
void motorCommandCallback(const std_msgs::Int16& motor_command);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> motor_command_subscriber("motor_commands", &motorCommandCallback);

void setup() {
  Serial.begin(9600); // Set baud rate to 9600
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(motor_command_subscriber);
  
  motorFL.setSpeed(MAX_SPEED);
  motorBL.setSpeed(MAX_SPEED);
  motorFR.setSpeed(MAX_SPEED);
  motorBR.setSpeed(MAX_SPEED);
}

void loop() {
  nh.spinOnce();
}

// Callback function for receiving motor commands
void motorCommandCallback(const std_msgs::Int16& motor_command) {
  int command = motor_command.data;

  // Adjust motor speeds based on the received command
  switch (command) {
    case 8: // Forward
      moveForward();
      break;
    case 2: // Backward
      moveBackward();
      break;
    case 4: // Left
      turnLeft();
      break;
    case 6: // Right
      turnRight();
      break;
    case 0: // Stop
      stopMoving();
      break;
    default:
      // Invalid command, stop moving
      stopMoving();
      break;
  }
}
void stopMoving() {
  motorBL.run(RELEASE);
  motorBR.run(RELEASE);
  motorFR.run(RELEASE);
  motorFL.run(RELEASE);
}

void moveForward() {
  motorBL.run(FORWARD);
  motorBR.run(FORWARD);
  motorFR.run(FORWARD);
  motorFL.run(FORWARD);
}

void moveBackward() {
  motorBL.run(BACKWARD);
  motorBR.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorFL.run(BACKWARD);
}

void turnRight() {
  motorBL.run(FORWARD);
  motorBR.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorFL.run(FORWARD);
}

void turnLeft() {
  motorBL.run(BACKWARD);
  motorBR.run(FORWARD);
  motorFR.run(FORWARD);
  motorFL.run(BACKWARD);
}
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, String

# Define the minimum distance threshold
MIN_DISTANCE = 30
# Define the maximum distance threshold
MAX_DISTANCE = 500

# Callback function to handle sensor data
def sensor_data_callback(msg):
    rospy.loginfo("Received sensor data: [%s]" % msg.data)
    # Parse the received data and make decision
    data = msg.data.split()
    distance_front = int(data[0])
    distance_left = int(data[1])
    distance_right = int(data[2])

    # Consider distance values greater than the maximum distance as 0
    distance_front = distance_front if distance_front <= MAX_DISTANCE else 0
    distance_left = distance_left if distance_left <= MAX_DISTANCE else 0
    distance_right = distance_right if distance_right <= MAX_DISTANCE else 0

    # Check if all three sensors are less than the MIN_DISTANCE
    if distance_front < MIN_DISTANCE and distance_left < MIN_DISTANCE and distance_right < MIN_DISTANCE:
        stop_moving()
    else:
        # Make decision based on sensor data
        if distance_front > MIN_DISTANCE:
            move_forward()
        else:
            if distance_left > distance_right:
                rotate_left()
            else:
                rotate_right()

# Function to publish motor commands
def publish_motor_command(command):
    rospy.loginfo("Published motor commands: [%s]" % command.data)
    publisher.publish(command)

# Function to move straight
def move_forward():
    publish_motor_command(Int16(8))

# Function to move backward
def move_backward():
    publish_motor_command(Int16(2))

# Function to rotate left
def rotate_left():
    publish_motor_command(Int16(4))

# Function to rotate right
def rotate_right():
    publish_motor_command(Int16(6))

# Function to stop moving
def stop_moving():
    publish_motor_command(Int16(0))

# Main function
def brain():
    global publisher
    rospy.init_node('brain', anonymous=True)
    publisher = rospy.Publisher('motor_commands', Int16, queue_size=10)
    rospy.Subscriber('sensor_data', String, sensor_data_callback)
    rospy.spin()

if __name__ == '__main__':
    brain()

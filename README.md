# ROS Arduino Project

This project demonstrates communication between two Arduino devices using ROS (Robot Operating System). The first Arduino is equipped with ultrasonic sensors and publishes sensor data, while the second Arduino controls motors based on commands received from the first Arduino via ROS messages.

## Hardware Requirements

- Arduino Uno (two units)
- Ultrasonic sensors
- Motor driver
- DC motors
- Jumper wires

## Software Requirements

- ROS (Robot Operating System)
- Arduino IDE

## Installation Instructions

1. Install ROS on your Ubuntu machine by following the instructions provided on the [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).

2. Set up the Arduino IDE for ROS by installing the required libraries:
   - ros_lib (provided by the rosserial_arduino package)
   - std_msgs

3. Clone this repository to your local machine.

4. Connect the first Arduino (sensors) to your computer via USB and upload the `listener_sensor.ino` sketch located in the `arduino1` directory.

5. Connect the second Arduino (motors) to your computer via USB and upload the `listener_motor.ino` sketch located in the `arduino2` directory.

6. Make sure to configure the correct serial port (e.g., `/dev/ttyACM0` for the first Arduino and `/dev/ttyACM1` for the second Arduino) in the Arduino IDE before uploading the sketches.

## Usage

1. **Launch ROS Core**: Open a terminal window and run the following command to start the ROS core:

    ```bash
    roscore
    ```
2. **Run ROS Node Brain.py**: Open a new terminal window and run the following command to start the ROS node which 
    ```bash
    rosrun my_listener_pkg brain.py
    ```

3. **Run ROS Node for Arduino 1**: Open a new terminal window and run the following command to start the ROS node for Arduino 1:

    ```bash
    rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=9600 _buffer_size:=512 __name:=serial_node_1
    ```

4. **Run ROS Node for Arduino 2**: Open another terminal window and run the following command to start the ROS node for Arduino 2:

    ```bash
    rosrun rosserial_python serial_node.py /dev/ttyACM1 _baud:=9600 _buffer_size:=512 __name:=serial_node_2
    ```

Make sure to replace `/dev/ttyACM0` and `/dev/ttyACM1` with the appropriate serial port for each Arduino.
    ```bash
    ls /dev/ttyACM*
    ```

Ensure that each node has a unique name specified with the `__name:=` argument to prevent conflicts. For example, `serial_node_1` and `serial_node_2`.

## Monitoring Echoed Data

You can monitor the data being transmitted to Arduino 2 by running the following commands in separate terminal windows:

```bash
rostopic echo /sensor_data
rostopic echo /motor_commands
```

These commands will display the messages being published on the respective ROS topics.

## Troubleshooting

- If you encounter any issues during installation or execution, refer to the official ROS documentation and community forums for assistance.

## Contributors

- Shival Gupta
- Dipshikha Chakroborty
- Jashan Uppal

## License

This project is licensed under the [MIT License](LICENSE).

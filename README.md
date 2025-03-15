
# Odrive Diff Drive

A very simple differential drive controller for use with the odrive
motor controller hardware and the ROS2 framework. Works by accepting
a desired twist and interfacing with the Odrive using a differential
drive controller to achieve it.

## Usage

**Assumes a linux/unix environment**   

First connect the odrive via usb or can. Then find the file in /dev that is linked to the
odrive communication channel. Pass this as the odrive_filepath parameter. Then measure
the size of the wheel radii and their distances from the center. Set the values of
l_wheel_rad, r_wheel_rad, and center_dist to these measured values. Now start the node.
Publishing to the cmd_vel topic that is set will result in the controller node writing
the appropriate rotational velocities to the odrive motors.

### Important!

On the Odrive controller, it is expected that the left wheel is motor 0 and the
right wheel is motor 1. If you do not conform to this, the controller will always
do the inverse of what you want, which won't work very well.

### Node Details

#### Parameters

| name                 | type   | default        | description                                                   |
|----------------------|--------|----------------|---------------------------------------------------------------|
| vel_in               | string | "cmd_vel"      | The topic to listen for velocity input on                     |
| publish_actual_vel   | bool   | true           | Whether to publish the velocity the odrive is currently at    |
| vel_out              | string | "real_cmd_vel" | The topic to publish the velocity the odrive is giving        |
| vel_publish_interval | double | 0.1            | The time interval in seconds to publish the odrive's velocity |
| l_wheel_rad          | double | 1.0            | The radius of the left wheel                                  | 
| r_wheel_rad          | double | 1.0            | The radius of the right wheel                                 | 
| center_dist          | double | 1.0            | The distance between the wheels along the axis of rotation    |
| odrive_filepath      | string | "/dev/tty*"    | The file to interface with the odrive on                      |

#### Topics

| type                | description                                                                                   |
|---------------------|-----------------------------------------------------------------------------------------------|
| geometry_msgs/Twist | Incoming velocity commands. Topic set by vel_in                                               |
| geometry_msgs/Twist | Outgoing velocity achieved. Topic set by vel_out. Only publishes if publish_actual_vel is set |

## References

* https://en.wikipedia.org/wiki/Differential_wheeled_robot
* https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html


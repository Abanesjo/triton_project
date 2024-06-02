#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import sys
import select
import termios
import tty

# Initialize global variables for linear and angular velocities
linear_velocity = 0.0
angular_velocity = 0.0
linear_speed = 1  # Define the linear speed
angular_speed = 1 # Define the angular speed
increment = 0.2  # Define the increment step for speed adjustments

# Define the publisher
pub = None

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_velocity():
    global linear_velocity, angular_velocity
    msg = Float32MultiArray()
    msg.data = [linear_velocity, angular_velocity]
    pub.publish(msg)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('velocity_control')
    pub = rospy.Publisher('/velocity', Float32MultiArray, queue_size=10)

    print("Control Your Robot!")
    print("---------------------------")
    print("Use WASD keys to control the velocities")
    print("Use +/- to increase/decrease linear speed")
    print("Use [] to increase/decrease angular speed")
    print("Press Q or ESC to quit")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            updated = False

            if key == 'w':
                if linear_velocity != linear_speed or angular_velocity != 0:
                    linear_velocity = linear_speed
                    angular_velocity = 0
                    updated = True
            elif key == 's':
                if linear_velocity != -linear_speed or angular_velocity != 0:
                    linear_velocity = -linear_speed
                    angular_velocity = 0
                    updated = True
            elif key == 'a':
                if angular_velocity != angular_speed or linear_velocity != 0:
                    angular_velocity = angular_speed
                    linear_velocity = 0
                    updated = True
            elif key == 'd':
                if angular_velocity != -angular_speed or linear_velocity != 0:
                    angular_velocity = -angular_speed
                    linear_velocity = 0
                    updated = True
            elif key == '=':
                linear_speed += increment
                print(f"Linear speed increased to: {linear_speed}")
            elif key == '-':
                linear_speed = max(0, linear_speed - increment)  # Ensure speed doesn't go negative
                print(f"Linear speed decreased to: {linear_speed}")
            elif key == '[':
                angular_speed += increment
                print(f"Angular speed increased to: {angular_speed}")
            elif key == ']':
                angular_speed = max(0, angular_speed - increment)  # Ensure speed doesn't go negative
                print(f"Angular speed decreased to: {angular_speed}")
            elif key == 'q' or key == '\x1b':  # Q or ESC key
                break
            else:
                if linear_velocity != 0 or angular_velocity != 0:
                    linear_velocity = 0
                    angular_velocity = 0
                    updated = True

            if updated:
                publish_velocity()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.signal_shutdown("Exit")
        print("\nExiting...")

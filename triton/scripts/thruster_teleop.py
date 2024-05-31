#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import sys
import select
import termios
import tty

# Define the default force magnitude
force_magnitude = 500.0

# Prompt user for force magnitude
try:
    force_magnitude = float(input("Enter the force magnitude: "))
except ValueError:
    print("Invalid input. Using default force magnitude of 500.")

# Define the force values for different keys
forces = {
    'w': [force_magnitude, force_magnitude],
    's': [-force_magnitude, -force_magnitude],
    'a': [-force_magnitude, force_magnitude],
    'd': [force_magnitude, -force_magnitude],
    'k': [0.0, 0.0]  # Define 'k' for the rest state
}

current_force = forces['k']

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_forces(force):
    global current_force
    if force != current_force:  # Only publish if the force has changed
        msg = Float32MultiArray(data=force)
        pub.publish(msg)
        current_force = force

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('/thruster_forces', Float32MultiArray, queue_size=10)

    print("Control Your Thrusters!")
    print("---------------------------")
    print("Forces are being published to /thruster_forces")
    print("Use WASD keys to move the thrusters")
    print("Press 'k' to stop (rest state)")
    print("Press 'q' to quit")

    try:
        publish_forces(forces['k'])  # Publish rest force initially
        while not rospy.is_shutdown():
            key = get_key()
            if key in forces:
                publish_forces(forces[key])
            elif key == 'q':
                break

    except Exception as e:
        print(e)

    finally:
        publish_forces(forces['k'])  # Ensure we return to rest state on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

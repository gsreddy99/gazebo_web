import os
import datetime

from flask import Flask, session, url_for, redirect, render_template, request, abort, flash

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import time
else:
  import tty

# Initialize the Flask webserver
app = Flask(__name__)

# Define the max linear velocity and max angular velocity.

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.03
ANG_VEL_STEP_SIZE = 0.1

# Initialize turtlebot3_web

rospy.init_node('turtlebot3_web')

# Initialize the publisher to publish to the topic cmd_vel to control the robot tb3_2
pub = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)

# Creating a turtlebot3 model
turtlebot3_model = rospy.get_param("model", "burger")

status = 0

# initialize targer linear velocity and angular velocity with 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

# calculate the linear velocity based on the model type
def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

# calculate the angular velocity based on the model type
def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

# print the linear velocity and angular velocity
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

# /up api to move the robot forward
@app.route("/up", methods = ["POST"])
def keyUp():
    print("Clicked on Up Button")
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.2
    status=0

    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
    print("Moving the robot forward %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    # publishing the twist object to the topic with updated linear and angular velocity
    pub.publish(twist)
    return "200"

# /down api to move the robot backward
@app.route("/down", methods = ["POST"])
def keyDown():

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
    print("Moving the robot backward %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1

    # Create a twist object
    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    # publishing the twist object to the topic with updated linear and angular velocity
    pub.publish(twist)
    return "200"

# /left api to move the robot anti clockwise
@app.route("/left", methods = ["POST"])
def keyLeft():

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
    print("Move the Robot anti clockwise %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1

    # Create a twist object
    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    # publishing the twist object to the topic with updated linear and angular velocity
    pub.publish(twist)
    return "200"

# /right api to move the robot clockwise
@app.route("/right", methods = ["POST"])
def keyRight():

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
    print("Moving the Robot clockwise %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1

    # Create a twist object
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    # publishing the twist object to the topic with updated linear and angular velocity
    pub.publish(twist)
    return "200"

# /reset api to move the robot to its base location
@app.route("/reset", methods = ["POST"])
def keyReset():

    target_linear_vel   = 0.0
    control_linear_vel  = 0.0
    target_angular_vel  = 0.0
    control_angular_vel = 0.0

    print("Move the Robot to its base location %s"%vels(target_linear_vel,target_angular_vel))

    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

    # publishing the twist object to the topic with updated linear and angular velocity
    pub.publish(twist)
    return "200"

@app.route("/")
def homePage():
    #rendering index.html as a base url
    return render_template("index.html")

if __name__ == "__main__":

    # Setting the application to run on port 9000
    app.run(host="localhost", port=9000, debug=True)

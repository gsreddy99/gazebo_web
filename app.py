import os
import datetime
import hashlib
from flask import Flask, session, url_for, redirect, render_template, request, abort, flash
import itsdangerous as it
import markupsafe as mk

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

app = Flask(__name__)
app.config.from_object('config')

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.03
ANG_VEL_STEP_SIZE = 0.1


rospy.init_node('turtlebot3_web')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

turtlebot3_model = rospy.get_param("model", "burger")

status = 0

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

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

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
    print("@@@@@@@@@@@@@@@@@ Up Before %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    pub.publish(twist)
    print("@@@@@@@@@@@@@@@@@ Up After %s"%vels(target_linear_vel,target_angular_vel))
    return "200"


@app.route("/down", methods = ["POST"])
def keyDown():
    print("Clicked on Down Button")

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
    print("@@@@@@@@@@@@@@@@@ Down Before %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1

    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    pub.publish(twist)
    print("@@@@@@@@@@@@@@@@@ Down After %s"%vels(target_linear_vel,target_angular_vel))
    return "200"

@app.route("/left", methods = ["POST"])
def keyLeft():
    print("Clicked on Left Button")
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
    print("@@@@@@@@@@@@@@@@@ Left Before %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1

    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    pub.publish(twist)
    print("@@@@@@@@@@@@@@@@@ Left After %s"%vels(target_linear_vel,target_angular_vel))
    return "200"

@app.route("/right", methods = ["POST"])
def keyRight():
    print("Clicked on Right Button")

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    LIN_VEL_STEP_SIZE = 0.03
    ANG_VEL_STEP_SIZE = 0.1
    status=0
    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
    print("@@@@@@@@@@@@@@@@@ Right Before %s"%vels(target_linear_vel,target_angular_vel))
    status = status + 1
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    pub.publish(twist)
    print("@@@@@@@@@@@@@@@@@ Right After %s"%vels(target_linear_vel,target_angular_vel))
    return "200"

@app.route("/reset", methods = ["POST"])
def keyReset():
    print("Clicked on Reset Button")

    target_linear_vel   = 0.0
    control_linear_vel  = 0.0
    target_angular_vel  = 0.0
    control_angular_vel = 0.0

    print("@@@@@@@@@@@@@@@@@ Reset Before %s"%vels(target_linear_vel,target_angular_vel))

    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)
    print("@@@@@@@@@@@@@@@@@ Reset After %s"%vels(target_linear_vel,target_angular_vel))
    return "200"

@app.route("/")
def homePage():
    return render_template("index.html")

if __name__ == "__main__":
    print('ItsDangerous version',it.__version__)
    print('Markup Version',mk.__version__)
    app.run(host="localhost", port=9000, debug=True)

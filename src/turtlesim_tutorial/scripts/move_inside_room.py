#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Define square room limits
X_MIN, X_MAX = 2.0, 15.0
Y_MIN, Y_MAX = 2.0, 15.0

robot_x = 0.0
robot_y = 0.0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot Position -> X: %f, Y: %f", robot_x, robot_y)

def move_inside_room(lin_vel, ang_vel):
    global robot_x, robot_y
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.init_node('move_inside_room', anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    vel = Twist()
    
    while not rospy.is_shutdown():
        if X_MIN <= robot_x <= X_MAX and Y_MIN <= robot_y <= Y_MAX:
            vel.linear.x = lin_vel
            vel.angular.z = ang_vel
            rospy.loginfo("Moving inside room")
        else:
            vel.linear.x = 0
            vel.angular.z = 0
            rospy.logwarn("Outside room limits! Stopping.")
        
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        v = rospy.get_param("~v", 1.0)  # Default linear velocity
        w = rospy.get_param("~w", 0.5)  # Default angular velocity
        move_inside_room(v, w)
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Define límites de la habitación cuadrada
X_MIN, X_MAX = 2.0, 8.0  # Ajustado a un tamaño más pequeño
Y_MIN, Y_MAX = 2.0, 8.0

robot_x = 0.0
robot_y = 0.0

def pose_callback(pose):
    """Actualiza la posición del robot con la información del tópico /turtle1/pose."""
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot Position -> X: %f, Y: %f", robot_x, robot_y)

def move_inside_room():
    """Mueve la tortuga dentro de los límites definidos."""
    global robot_x, robot_y
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rate = rospy.Rate(10)  # 10 Hz
    vel = Twist()
    
   
    while not rospy.is_shutdown():
        if X_MIN <= robot_x <= X_MAX and Y_MIN <= robot_y <= Y_MAX:
            vel.linear.x = v
            vel.angular.z = w
            rospy.loginfo("Moving inside room")
        else:
            vel.linear.x = -vel.linear.x  # Invertir dirección
            vel.angular.z = -vel.angular.z  # Invertir giro
            rospy.logwarn("Outside room limits! Changing direction.")
        
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_inside_room', anonymous=False)
        v = rospy.get_param("~v", 0.5)  # Velocidad lineal reducida
        w = rospy.get_param("~w", 0.25)  # Velocidad angular reducida

        move_inside_room()
    except rospy.ROSInterruptException:
        pass

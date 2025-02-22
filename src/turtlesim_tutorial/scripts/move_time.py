#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n", pose.x, pose.y)

def move_turtle(lin_vel, ang_vel, move_time):
    global robot_x, robot_y
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)  # 10hz
    vel = Twist()

    # Calcular el tiempo final
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time

        # Establecer la velocidad
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel

        # Publicar el mensaje de velocidad
        pub.publish(vel)

        # Verificar si ha pasado el tiempo especificado
        if elapsed_time >= move_time:
            rospy.loginfo("Time interval reached, stopping the robot")
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_time', anonymous=False)  # Inicializar el nodo aquí al principio
        v = rospy.get_param("~v")  # Velocidad lineal
        w = rospy.get_param("~w")  # Velocidad angular
        t = rospy.get_param("~t")  # Tiempo de movimiento en segundos
        move_turtle(v, w, t)
    except rospy.ROSInterruptException:
        pass

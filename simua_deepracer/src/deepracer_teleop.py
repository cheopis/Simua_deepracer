
import rclpy
import numpy as np

from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from math import sqrt, sin, cos, tan, atan2
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


global pos

class MoveRobot(Node):
    def __init__(self):
        super().__init__('publisher')
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odomCallBack, 10)
        self.platformPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_robot = Twist()

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.go_to_goal = self.create_timer(0.1, self.goToGoal)
        self.flag = True
    
    def odomCallBack(self, data):
        global pos
        quat = data.pose.pose.orientation
        q = np.array([quat.x, quat.y, quat.z, quat.w])
        theta = euler_from_quaternion(q, 'sxyz')[2]

        pos = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            theta
            ])
        
    def stopRobot(self):
        self.move_robot.linear.x = 0.0
        self.move_robot.linear.y = 0.0
        self.move_robot.angular.z = 0.0
        self.platformPublisher.publish(self.move_robot)

    def timer_callback(self):
        if self.flag:
            self.getTeleop()
    
    def goToGoal(self):
        global pos
        try:
            linear_x = self.goal[0] - pos[0] 
            linear_y = self.goal[1] - pos[1]
                       
            distance = abs(sqrt((linear_x**2) + (linear_y**2)))
            if distance > 0.05:

                P_linear = 0.3
                P_angular = 0.1

                I_linear = 0.0
                I_angular = 0.01

                Kpp = 1

                phi = atan2(linear_y,linear_x) - pos[2] # self.goal[2] +

                if phi > np.pi/2:
                    phi -= np.pi
                if phi < - np.pi/2:
                    phi += np.pi 

                # linear_x_speed = linear_x
                # linear_y_speed = linear_y
                linear_x_speed = cos(pos[2]) * linear_x + sin(pos[2]) * linear_y
                linear_y_speed = cos(pos[2]) * linear_y + sin(pos[2]) * linear_x
                angular_speed = tan(phi)/15 * linear_x_speed

                steer_output=np.arctan(2*3*np.sin(phi)/(Kpp*linear_x_speed))

                self.move_robot.linear.x = (linear_x_speed * P_linear + 
                                            distance * I_linear)
                self.move_robot.linear.y = (linear_y_speed * P_linear + 
                                            distance * I_linear)
                self.move_robot.angular.z = steer_output * P_angular#(angular_speed * P_angular +
                                            #(distance * phi) * I_angular)

                self.platformPublisher.publish(self.move_robot)
            else:
                self.stopRobot()
                self.go_to_goal.cancel()
                self.flag = True
        except:
            self.stopRobot()
            print('error \n stopping movement timer')
            self.stopRobot()
            self.go_to_goal.cancel()
            self.flag = True
        

    def getTeleop(self):
        self.flag = False

        x_input = float(input('Set a x axis goal for the robot:'))
        y_input = float(input('Set a y axis goal for the robot:'))
        #theta_input = float(input('Set a theta axis, in radians, goal for the robot:'))

        self.goal = [x_input, y_input] #, theta_input]

        self.go_to_goal.reset()


def main(args=None):
    rclpy.init(args=args)
    publisher = MoveRobot()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
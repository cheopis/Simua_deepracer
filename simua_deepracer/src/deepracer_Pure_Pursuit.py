
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
        self.Ed = 0
    
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

    def clamp(self, n, minn, maxx):
        if n < minn:
            return minn
        elif n > maxx:
            return maxx
        else:
            return n
    
    def goToGoal(self):
        global pos
        try:
            linear_x = self.goal[0] - pos[0] 
            linear_y = self.goal[1] - pos[1]
                       
            distance = abs(sqrt((linear_x**2) + (linear_y**2)))
            if distance > 0.05:

                P_linear = 0.3
                P_angular = 0.1

                I_linear = 0.001
                I_angular = 0.01

                Kpp = 1

                phi = atan2(linear_y,linear_x) - pos[2]

                linear_x_speed = cos(pos[2]) * linear_x + sin(pos[2]) * linear_y
                linear_y_speed = cos(pos[2]) * linear_y + sin(pos[2]) * linear_x

                steer_output=np.arctan(2*3*np.sin(phi)/(Kpp*linear_x_speed))

                x_speed = self.clamp(linear_x_speed * P_linear
                                     + self.Ed_x * I_linear,
                                     -10.0,
                                     10.0)
                y_speed = self.clamp(linear_y_speed * P_linear
                                     + self.Ed * I_linear,
                                     -10,
                                     10)
                w_speed = self.clamp(steer_output * P_angular,
                                     -1.2,
                                     1.2)
                

                self.move_robot.linear.x = x_speed  
                self.move_robot.linear.y = y_speed
                self.move_robot.angular.z = w_speed

                self.platformPublisher.publish(self.move_robot)

                self.Ed_x += linear_x_speed
                # self.Ed_y += linear_y_speed
                
            else:
                self.Ed_x = 0
                self.stopRobot()
                self.go_to_goal.cancel()
                self.flag = True
        except:
            self.stopRobot()
            self.Ed_x = 0
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
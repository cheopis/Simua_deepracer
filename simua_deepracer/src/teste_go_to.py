import rclpy

from rclpy.node import Node
from deepracer_teleop import MoveRobot
from geometry_msgs.msg import Twist
from math import sqrt
from threading import Thread

class userInterface(Node):
    def __init__(self) -> None:
        super().__init__('teleop')
        Thread(target = MoveRobot.startApp).start()
        self.platformPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_robot = Twist()
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.flag = True

    def getPose(self):
        pass

    def timer_callback(self):
        if self.flag:
            self.getTeleop()
    

    def goToGoal(self, x_goal, y_goal, theta_goal = 0.0):
        distance = 1000
        try:
            while(distance > 0.02):
                K_linear = 0.2
                distance = abs(sqrt(((x_goal - self.pos[0])**2) + ((y_goal - self.pos[1])**2)))

                linear_x_speed = (x_goal - self.pos[0])*(distance * K_linear)
                linear_y_speed = (y_goal - self.pos[1])*(distance * K_linear)
                
                K_angular = 4.0
                angular_speed = (theta_goal - self.pos[2]) * K_angular

                self.move_robot.linear.x = linear_x_speed
                self.move_robot.linear.y = linear_y_speed
                self.move_robot.angular.z = angular_speed

                self.platformPublisher.publish(self.move_robot)
            return True
        except:
            print('error')

    def getTeleop(self):
        self.flag = False
        x_input = float(input('Set a x axis goal for the robot:'))
        y_input = float(input('Set a y axis goal for the robot:'))
        theta_input = float(input('Set a theta axis goal for the robot:'))
        
        flag = self.goToGoal(x_input, y_input, theta_input)
        print('ok')
        print(self.flag)
        self.flag = flag

def main(args=None):
    rclpy.init(args=args)
    
    teleop = userInterface()
    rclpy.spin(teleop)

if __name__ == '__main__':
    main()
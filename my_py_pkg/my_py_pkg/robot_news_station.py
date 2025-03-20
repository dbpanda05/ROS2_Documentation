#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name_="C3PO"
        self.publisher_=self.create_publisher(String, "robot_news", 10) #This converts a node to a publisher
        self.timer_=self.create_timer(0.5, self.publish_news)           #This creates a timer to publish news every 0.5 seconds
        self.get_logger().info("Robot News Station has been started")   #This logs a message to the console before it starts publishing news

    def publish_news(self):                                             #This is the function that will be called every 0.5 seconds
        msg=String()                                                    #This is the message type that will be published
        msg.data="Hi, this is " + self.robot_name_+" from the robot news station."                                          #This is the message that will be published
        self.publisher_.publish(msg)                                    #This publishes the message

def main(args=None):
    rclpy.init(args=args)
    node=RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()   
        
#!/usr/bin/env python3

import os
import signal
import rclpy
from rclpy.node import Node
from motor_demo_msgs.msg import MotorCommand
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)


class CommandPubNode(Node):
    def __init__(self):
        super().__init__("MotorPublisher")
        self.get_logger().info("Listener has been started")
        self.mot_cmd_pub_ = self.create_publisher(MotorCommand, "motor_cmd", 10)
        self.get_logger().info(
            f"""
        Controls:
        WASD or Arrows to move
        Any other key to stop
        CTRL-C or q to quit
        """
        )
        try:
            while(1):
                key = self.getKey()
                self.on_press(key)
                if (key == '\x03'):
                    break
        except:
            print("Something went wrong when writing to the file")
        finally:
            self.on_press(key)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def on_press(self, key):
        self.get_logger().info("Debug")
        msg = MotorCommand()
        msg.direction = 'no'
        if key == "w":
            msg.direction = 'forward'
        elif key == "s":
            msg.direction = 'backward'
        if key == "a":
            msg.turn = 'left'
        elif key == "d":
            msg.turn = 'right'
        if key == "q":
            os.kill(os.getpid(), signal.SIGINT)
        self.mot_cmd_pub_.publish(msg)

def main():
    rclpy.init()
    node = CommandPubNode()
    rclpy.spin(node)
    rclpy.shutdown()




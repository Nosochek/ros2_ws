import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from motor_demo_msgs.msg import MotorCommand


class MotorDriver(Node):
    Motor_A_EN = 4
    Motor_B_EN = 17
    Motor_A_Pin1 = 14
    Motor_A_Pin2 = 15
    Motor_B_Pin1 = 27
    Motor_B_Pin2 = 18
    Dir_forward = 0
    Dir_backward = 1
    left_forward = 0
    left_backward = 1
    right_forward = 0
    right_backward = 1
    pwn_A = 0
    pwm_B = 0

    def __init__(self):
        super().__init__('motor_driver')
        self._msg = MotorCommand()
        self.setup()
        self.subscription = self.create_subscription(
            MotorCommand, 'motor_cmd', self.motor_command_callback, 10)
    def motor_command_callback(self, msg: MotorCommand):
        self._msg = msg
        self.get_logger().info("(" + str(msg.direction) + ", " + str(msg.turn) + ")")
        self.move(100, self._msg.direction, self._msg.turn)


    def setup(self):  # Motor initialization
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)
        self.motorStop()
        try:
            self.pwm_A = GPIO.PWM(self.Motor_A_EN, 1000)
            self.pwm_B = GPIO.PWM(self.Motor_B_EN, 1000)
        except:
            pass

    def motorStop(self):  # Motor stops
        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)

    def motor_left(self, status, direction, speed):  # Motor 2 positive and negative rotation
        if status == 0:  # stop
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_B_EN, GPIO.LOW)
        else:
            if direction == self.Dir_backward:
                GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
                self.pwm_B.start(100)
                self.pwm_B.ChangeDutyCycle(speed)
            elif direction == self.Dir_forward:
                GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
                self.pwm_B.start(0)
                self.pwm_B.ChangeDutyCycle(speed)

    def motor_right(self, status, direction, speed):  # Motor 1 positive and negative rotation
        if status == 0:  # stop
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_A_EN, GPIO.LOW)
        else:
            if direction == self.Dir_forward:  #
                GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
                self.pwm_A.start(100)
                self.pwm_A.ChangeDutyCycle(speed)
            elif direction == self.Dir_backward:
                GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)
                self.pwm_A.start(0)
                self.pwm_A.ChangeDutyCycle(speed)
        return direction

    def move(self, speed, direction, turn, radius=0.6):  # 0 < radius <= 1
        # speed = 100
        self.get_logger().info("Move")
        if direction == 'forward':
            if turn == 'right':
                self.get_logger().info("forward right")
                self.motor_left(0, self.left_backward, int(speed * radius))
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.get_logger().info("forward left")
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(0, self.right_backward, int(speed * radius))
            else:
                self.get_logger().info("forward straight")
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                self.get_logger().info("backward right")
                self.motor_left(0, self.left_forward, int(speed * radius))
                self.motor_right(1, self.right_backward, speed)
            elif turn == 'left':
                self.get_logger().info("backward left")
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(0, self.right_forward, int(speed * radius))
            else:
                self.get_logger().info("backward straight")
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_backward, speed)
        elif direction == 'no':
            if turn == 'right':
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_backward, speed)
            else:
                self.get_logger().info("attempt to stop")
                self.motorStop()
        else:
            pass
        
        self._msg.direction = 'no'

    def destroy(self):
        self.motorStop()
        GPIO.cleanup()



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()

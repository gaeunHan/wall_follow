import rclpy
from rclpy.node import Node
import serial
import math

import numpy as np
from sensor_msgs.msg import LaserScan
#from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create laser topic subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10)
        self.subscriber # prevent unused variable warning

        '''
        # create drive topic publisher
        self.publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pid_control)
        self.publisher # prevent unused variable warning
        '''
        
        # create serial object
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # set PID gains
        self.kp = 14
        self.kd = 0.09
        self.ki = 0

        # init history variables
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # init lidar variable
        self.frontDistLiDAR = 0.0   
        self.rightDistLiDAR = 0.0  
        self.leftDistLiDAR = 0.0

        # constants
        self.L_lookAhead = 1.5          # meters
        self.throttle_max = 60          # fastest throttle
        self.steering_max = 1           # rightest steering
        self.LiDAR_detect_max = 10      # meters
        self.refDistToRightWall = 0.5   # meters

    def toRadian(self, degree):
        """
        Convert angle to radian from degree
        """
        return float(math.pi * degree / 180.0)


    def get_range(self, msg, angle):
        """
        Get range[] data to desired angle[rad]
        """
        idx = int((angle - msg.angle_min) / msg.angle_increment)
        distance = msg.ranges[idx]

        # saturation
        if(math.isinf(distance) or math.isnan(distance)):
            distance = self.LiDAR_detect_max

        return distance

    def get_error(self, a, b, theta):
        """
        Calculates the error to the wall. Follow the wall on the right
        """
        alpha = math.atan((a * math.cos(theta) - b)/(a*math.sin(theta)))
        currDistToRightWall = b*math.cos(alpha)

        currErr = self.refDistToRightWall - currDistToRightWall

        # advanced
        aheadDistToRightWall = currDistToRightWall + self.L_lookAhead * math.sin(alpha)

        return currErr

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control
        """

        error = self.get_error()
        self.integral = self.integral + error

        # make a steering control input
        angle = self.kp * error + self.kd * (error - self.prev_error) + self.ki * self.integral

        self.prev_error = error 

        # determine throttle speed depends on the amount of steering control input 
        if angle >= abs(self.toRadian(0)) and angle < abs(self.toRadian(10)):
            velocity = self.throttle_max
        elif angle >= abs(self.toRadian(10)) and angle < abs(self.toRadian(20)):
            velocity = self.throttle_max / 2
        else:
            velocity = self.throttle_max / 4

        '''
        drive_msg = AckermannDriveStamped()
        drive_msg.steering_angle = angle
        drive_msg.speed = velocity
        self.publisher.publish(drive_msg)
        '''

        return angle, velocity
    
    def send_control_to_car(self, steering, throttle):
        self.ser.write(bytearray([throttle, steering])) # throttle, steering 값을 serial 통신을 통해 보냅니다.
        self.get_logger().info('Sending: Throttle=%d, Steer=%d' % (throttle, steering)) # throttle, steering 값을 로그에 출력합니다.

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        """
        a = self.get_range(self.toRadian(msg, -55.0))
        b = self.get_range(self.toRadian(msg, -90.0))
        theta = 90.0 - 55.0

        # calc dist error
        error = self.get_error(a, b, theta)

        # make control input
        angle, velocity = self.pid_control(error) 

        # drive the car 
        self.send_control_to_car(angle, velocity)

        

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

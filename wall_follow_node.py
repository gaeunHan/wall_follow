import rclpy
from rclpy.node import Node
import serial
import math

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

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


        # create drive topic publisher
        self.publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pid_control)
        self.publisher # prevent unused variable warning
        
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
        self.pi = 3.14              # pi in radian
        self.L_lookAhead = 1.5      # meters
        self.throttle_max = 60      # fastest throttle
        self.steering_max = 1       # rightest steering
        self.LiDAR_detect_max = 10  # meters


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        span = angle
    
        #TODO: implement
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.steering_angle = ??
        drive_msg.speed = ??
        self.publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        frontAngle = 0.0
        leftestAngle = self.pi/2
        rightestAngle = -self.pi/2

        frontIdx = self.get_range(frontAngle)
        leftIdx = self.get_range(leftestAngle)
        rightIdx = self.get_range(rightestAngle)
        
        self.frontDistLiDAR = msg.ranges[frontIdx]
        self.leftDistLiDAR = msg.ranges[leftIdx]
        self.rightDistLiDAR = msg.ranges[rightIdx]

        # inf handling
        if(math.isinf(self.frontDistLiDAR)):
            self.frontDistLiDAR = self.LiDAR_detect_max
        if(math.isinf(self.leftDistLiDAR)):
            self.leftDistLiDAR = self.LiDAR_detect_max
        if(math.isinf(self.rightDistLiDAR)):
            self.rightDistLiDAR = self.LiDAR_detect_max

        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID
        

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
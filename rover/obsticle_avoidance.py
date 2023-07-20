import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('go_to_position_node')
        # Subscriber to the scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.get_scan_values, 40)

        # Constant Velocity Value, contains a more constant velocity that we can call to to keep things consistent
        self.linear_vel = 0.22

        # Split up the regions of the laser scanner
        self.regions = {'right': [], 'front': [], 'left': []}

        # Publisher to the cmd_vel topic
        self.velocity = Twist()

    def get_scan_values(self, scan_data):
        # LOGIC EXPLAINATION
        # We are going to split up the laser scanner into 3 regions, right, front, and left
        # We have 360 values to divide over the front of the robot, if there is nothing in front
        # of the scanner the value will be infinty. So we can say something is in the region
        # if the minimum value is less than 100, as this will always be less than infinity
        # and anything in the region will always return a value less than 100

        self.regions = {
            'right': min(min(scan_data.ranges[0:120]), 100),
            'front': min(min(scan_data.ranges[120:240]), 100),
            'left': min(min(scan_data.ranges[240:360]), 100)
        }
        print("Lidar Data:\t", self.regions['left'],
              "\t", self.regions['front'],
              "\t", self.regions['right'])


def main():
    rclpy.init(args=None)
    obsticle_avoidance = ObstacleAvoidingBot()
    rclpy.spin(obsticle_avoidance)
    rclpy.shutdown()

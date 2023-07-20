import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('go_to_position_node')
        # Publisher for comand velocity topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40) # Twist contains linear and angular velocities and publishes it periodically
        # Subscriber to the scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.get_scan_values, 40)
        
        # Timer to send velocity commands
        timer_period = 0.2;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
         
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
    
    def send_cmd_vel(self):
        self.velocity.linear.x = self.linear_vel # Set the linear velocity in the x direction (straight ahead) to be the parameter we set earlier
        
        if (self.regions['left'] > 4 and self.regions['front'] > 4 and self.regions['right'] > 4):
            self.velocity.angular.z = 0.0 # If there is nothing in front of the robot, set the angular velocity about the z axis to 0
            print("going straight")
            
        elif (self.regions['left'] > 4 and self.regions['front'] > 4 and self.regions['right'] < 4):
            self.velocity.angular.z = 1.57 # If there is something to the right of the robot, set the angular velocity about the z axis to 1.57 (turn left)
            print("turning left")
            
        elif (self.regions['left'] < 4 and self.regions['front'] > 4 and self.regions['right'] > 4):
            self.velocity.angular.z = -1.57 # If there is something to the left of the robot, set the angular velocity about the z axis to -1.57 (turn right)
            print("turning right")
            
        elif (self.regions['left'] < 4 and self.regions['front'] < 4 and self.regions['right'] < 4) or (self.regions['left'] > 4 and self.regions['front'] < 4 and self.regions['right'] > 4):
            self.velocity.angular.z = 3.14 # If there is something in front of the robot, set the angular velocity about the z axis to 3.14 (turn around)
            self.velocity.linear.x = -self.linear_vel # Set the linear velocity in the x direction (straight ahead) to be 0
            print("turning around")
        
        self.publisher.publish(self.velocity) # Publish the velocity to the cmd_vel topic
        


def main():
    rclpy.init(args=None)
    obsticle_avoidance = ObstacleAvoidingBot()
    rclpy.spin(obsticle_avoidance)
    rclpy.shutdown()

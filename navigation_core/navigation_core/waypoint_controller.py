 #!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import  Quaternion
from rclpy.executors import MultiThreadedExecutor


from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, sqrt, degrees
import numpy
import time
import serial


lat1=0; lon1=0; lat2=0; lon2=0; quat=0; robotStatus=0; targetStatus=0;
f1 = 0
path_id = 0

rover_heading = 0.0
ref_heading = 10.00
heading_error_i = 0.0

gps1lat = 0.0
gps1lon = 0.0
gps2lat = 0.0
gps2lon = 0.0
rover_lat = 0.0
rover_lon = 0.0

ref_coord_1_lat = 0.1
ref_coord_1_lon = 0.12
ref_coord_2_lat = 0.13
ref_coord_2_lon = 0.14

history = []

#latitudes_field   = [37.260939600, 37.260467900]
#longitudes_field = [-121.839533600, -121.839519100]

latitudes_field   = [ref_coord_1_lat, ref_coord_2_lat]
longitudes_field = [ref_coord_1_lon, ref_coord_2_lon]

class getGPS(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
		NavSatFix,
		'robot1/gps1',
        self.listener_callback,
		5)
        self.subscription

    def listener_callback(self, msg):
        global lat1,lon1,robotStatus
        robotStatus = msg.status.status
        lat1 = msg.latitude
        lon1 = msg.longitude
        print("Lat, Lon: ", lat1, lon1)

class getQuat(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
		Quaternion,
		'robot1/imu/quaternion',
        self.listener_callback,
		5)
        
        self.subscription

    def listener_callback(self, msgQ):
        global yawRover
        quat = [msgQ.x, msgQ.y, msgQ.z, msgQ.w]
        yawRover = atan2(2.0*(quat[1]*quat[2] + quat[3]*quat[0]), quat[3]*quat[3] - quat[0]*quat[0] - quat[1]*quat[1] + quat[2]*quat[2]);
        print("yaw:", yawRover)
class getTarget(Node):

    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
		NavSatFix,
		'/robot1/target',
        self.listener_callback,
		5)
        self.subscription

    def listener_callback(self, msg):
        global targetStatus,lat2,lon2
        targetStatus = msg.status.status
        lat2 = msg.latitude
        lon2 = msg.longitude

class giveDirections(Node):

    def __init__(self):
        super().__init__('directions_publisher')
        self.publisher_ = self.create_publisher(
        	Twist,
        	'/robot1/cmd_vel', 
        	5)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        bearingX = cos(lat2) * sin(lon2-lon1)
        bearingY = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1)
        yawTarget = atan2(bearingX,bearingY)
        yawTarget = 3.14/2
        yawDelta = yawTarget - yawRover
        
        dist = sqrt((lat2-lat1)**2 + (lon2-lon1)**2)
        
        msg = Twist()
        if dist > 0:
            msg.linear.x = 0.7
            msg.angular.z = 0.5 * degrees(yawDelta)/360
        msg.linear.x=0.7;
        msg.angular.z=0.2;

        #print(msg)
        self.publisher_.publish(msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    Quat = getQuat()
    gpsR = getGPS()
    Target = getTarget()
    Directions = giveDirections()

    executor = MultiThreadedExecutor()
    executor.add_node(Quat)
    executor.add_node(Directions)
    executor.add_node(gpsR)
    executor.add_node(Target)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

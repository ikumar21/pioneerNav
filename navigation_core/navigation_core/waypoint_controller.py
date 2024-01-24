 #!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, sqrt, degrees
import numpy
import time
import serial


iniDesiredCoor = [37.35161161607423, -121.94109270507805]
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

class giveDirections(Node):

    def __init__(self):

        # Publisher to give cmd_vel to movebase_kinematics (linear x, angular z)
        super().__init__('directions_publisher')
        self.publisher_ = self.create_publisher(
        	Twist,
        	'/robot1/cmd_vel', 
        	5)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.give_dir)

        # Create a subscription to get current Euler Angles from IMU
        self.curHeading = 0; #Euler Angle (heading)
        self.subscriptionEuler = self.create_subscription(
            Float32MultiArray,
            'robot1/imu/eulerAngle',
            self.euler_callback, 
            5)
        self.subscriptionEuler 

        # Create a subscription to get current location from GPS
        self.curLat = 0.0;
        self.curLon = 0.0;
        self.statusGPS=True;
        self.subscriptionLoc = self.create_subscription(
		NavSatFix,
		'robot1/gps1',
        self.current_gps_callback,
		5)
        self.subscriptionLoc

        # # Create a subscription to get desired GPS location
        self.desLat, self.desLon  = iniDesiredCoor;
        self.subscriptionTarget = self.create_subscription(
		NavSatFix,
		'/robot1/target',
        self.target_callback,
		5)
        self.subscriptionTarget

        self.i = 0
    def current_gps_callback(self, msgC:NavSatFix):
        self.statusGPS = msgC.status.status
        print("Status GPS:", self.statusGPS , "Lat/Lon:", msgC.latitude,msgC.longitude)
        if(self.statusGPS!=0):
            self.curLat = msgC.latitude
            self.curLon = msgC.longitude

    def target_callback(self, msgD:NavSatFix):

        print("Desired Lat/Lon:", msgD.latitude,msgD.longitude)
        self.desLat = msgD.latitude
        self.desLon = msgD.longitude


    def euler_callback(self, msgE:Float32MultiArray):
        self.curHeading=msgE.data[0];
        print("Current Heading:",self.curHeading)
    def give_dir(self):
        bearingX = cos(self.desLat) * sin(self.desLon-self.curLon)
        bearingY = cos(self.curLat) * sin(self.desLat) - sin(self.curLat) * cos(self.desLat) * cos(self.desLon-self.curLon)
        yawTarget = atan2(bearingX,bearingY)
        print("Target bearing", degrees(yawTarget))
        yawTarget = 90
        yawDelta = degrees(yawTarget) - self.curHeading
        
        dist = sqrt((self.desLat-self.curLat)**2 + (self.desLon-self.curLon)**2)
        
        msg = Twist()
        if dist > 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * yawDelta/360
            print("dist", dist)
        msg.linear.x=0.0;
        msg.angular.z=0.2;

        #print(msg)
        self.publisher_.publish(msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    Directions = giveDirections()
    rclpy.spin(Directions)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Directions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

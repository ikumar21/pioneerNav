import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

import time
import board
import adafruit_bno055
import yaml

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

import math

def set_calibration(sensor:adafruit_bno055.BNO055_I2C, calData:list):
    """Set the sensor's calibration data using a list of 22 bytes that
    represent the sensor offsets and calibration data.  This data should be
    a value that was previously retrieved with get_calibration (and then
    perhaps persisted to disk or other location until needed again).
    """

    # Check that 22 bytes were passed in with calibration data.
    if calData is None or len(calData) != 22:
        print(calData)
        raise ValueError('Expected a list of 22 bytes for calibration data.')
    
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
    time.sleep(0.02)  # Datasheet table 3.6

    # Write 22 bytes of calibration data.
    registerAddr = 0x6A #Start with Magnometer radius MSB register

    for i in range(22):
            sensor._write_register(registerAddr,calData[i])
            #Update register Address:
            registerAddr-=0x1;

    # Go back to normal operation mode.
    time.sleep(0.01)  # Table 3.6
    sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)


def get_calibration(sensor:adafruit_bno055.BNO055_I2C):
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the set_calibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
        time.sleep(0.02)  # Datasheet table 3.6

        # Read the 22 bytes of calibration data and put it in a list
        calData = [];
        registerAddr = 0x6A #Start with Magnometer radius MSB register

        for i in range(22):
             calData.append(sensor._read_register(registerAddr))
             #Update register Address:
             registerAddr-=0x1;

        # Go back to normal operation mode.
        time.sleep(0.01)  # Table 3.6
        sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)
        return calData


class readIMU(Node):
    def __init__(self):
        super().__init__('robot1_imu')
        self.declare_parameters(
             namespace='',
             parameters=[
                  ('calibrateIMU',None),
                  ('calibOffsetsRadii',None),
                  ('calibFileLoc',None)
             ]
        )
        self.publisherQ_ = self.create_publisher(Quaternion, 'robot1/imu/quaternion', 1)
        self.publisherE_ = self.create_publisher(Float32MultiArray, 'robot1/imu/eulerAngle', 3)
        self.publisherC_ = self.create_publisher(Int16MultiArray, 'robot1/imu/calibInfo', 4)

        # Create a subscription to its own topic with a callback function 
        #This is so it can listen for commands to start/set/store calibration
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'robot1/imu/calibCom',
            self.calib_cmd_callback, 
            5)
        self.subscription  

        #Set Calibration if calibIMU is True
        if(self.get_parameter('calibrateIMU').get_parameter_value()):
            calibDataParam = self.get_parameter('calibOffsetsRadii').get_parameter_value().integer_array_value
            set_calibration(sensor,calibDataParam)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def calib_cmd_callback(self, msg:Int16MultiArray):
        global sensor
        runCalibResetDevice = msg.data[0]
        setCalibParam = msg.data[1]
        storeCalib = msg.data[2]

        # Only do something if command
        if(runCalibResetDevice):
            # Run Calibration by resetting device
            sensor = adafruit_bno055.BNO055_I2C(i2c)

        if(setCalibParam and not runCalibResetDevice):
            # Set Calibration Offsets/Radii from parameters file
            calibData = self.get_parameter('calibOffsetsRadii').get_parameter_value().integer_array_value
            set_calibration(sensor,calibData)

        if(storeCalib and not setCalibParam and not runCalibResetDevice):
            #Store current calib offsets/radii in parameters file
            #Also set parameter CalibIMU to True so offsets will be used next time
            calData=get_calibration(sensor)

            #Store Data
            calibFile = self.get_parameter('calibFileLoc');
            calibFile=calibFile.get_parameter_value().string_value;
            data = {'robot1_imu':{'ros__parameters':{'calibrateIMU': 1, 'calibOffsetsRadii':calData,'calibFileLoc':calibFile}}}

            with open(calibFile,'w',) as yamlFile:
                yaml.dump(data,yamlFile,default_flow_style=None)
                yamlFile.close()
             

    def timer_callback(self):
        

        #Publish Quaternion Data
        msgQ = Quaternion()
        msgQ.x, msgQ.y, msgQ.z, msgQ.w = sensor.quaternion[0], sensor.quaternion[1], sensor.quaternion[2], sensor.quaternion[3]
        self.publisherQ_.publish(msgQ)

        #Publish Euler Angle Data
        msgE = Float32MultiArray()
        msgE.data=sensor.euler
        self.publisherE_.publish(msgE)

        #Publish Calibration Status
        #One Array: [runCalibResetDevice setCalibrationParam storeCalib sysCalib gyroCalib accelCalib magCalib]
        #Array Data Range: [0/1 0/1 0/1 0-3 0-3 0-3 0-3]

        msgC = Int16MultiArray()
        # msgC.data = [0,0,0]
        # msgC.data.extend(sensor.calibration_status)
        msgC.data=sensor.calibration_status
        self.publisherC_.publish(msgC)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    get_imu_data = readIMU()

    rclpy.spin(get_imu_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_imu_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
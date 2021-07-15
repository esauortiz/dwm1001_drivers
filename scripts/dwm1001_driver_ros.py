#!/usr/bin/env python

""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import os, sys
from pickle import FALSE
import rospy, time, serial, random
from std_msgs.msg import Header
from std_msgs.msg       import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from uwb_msgs.msg import AnchorInfo

from dwm1001_apiCommands import DWM1001_API_COMMANDS

#TODO
#from sensor_msgs.msg import Imu

class ReadyToLocalize(object):


    def __init__(self, anchors, do_ranging_attempts, world_frame_id, tag_frame_id, tag_device_id, algorithm=None, dimension=None, height=1000):
        """
        Initialize serial port
        """
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.range_error_counts = [0 for i in range(len(self.anchors))]
        self.world_frame_id = world_frame_id
        self.tag_frame_id = tag_frame_id
        self.tag_device_id = tag_device_id
        self.do_ranging_attempts = do_ranging_attempts

        # Get port and tag name
        self.dwm_port = rospy.get_param('~serial_port')
        self.use_network = rospy.get_param('~use_network', False)
        self.network = rospy.get_param('~network', "default")
        self.verbose = rospy.get_param('~verbose', False)

        # Empty dictionary to store topics being published
        self.topics = {}
        
    def initSerial(self):
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # Serial port settings
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS,
            timeout = 0.05
        )

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

    def initializeDWM1001API(self):
        """
        Initialize dwm10801 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def readSerial(self):
        """
        Read serial string and return data as array
        :param:
        :returns: data array
        """
        try:
            serial_read_line = self.serialPortDWM1001.read_until()
        except:
            return ['']
        array_data = [x.strip() for x in serial_read_line.strip().split(' ')]
        return array_data

    def handleKeyboardInterrupt(self):
        """
        Handles keyboard interruption
        :param:
        :returns: none
        """
        rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def quit(self):
        """
        QUit and send reset command to dev board
        :param:
        :returns: none
        """
        rospy.loginfo("Quitting, and sending reset command to dev board")
        # self.serialPortDWM1001.reset_input_buffer()
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(1.0)
        serialReadLine = self.serialPortDWM1001.read_until()
        if "reset" in serialReadLine:
            rospy.loginfo("succesfully closed ")
            self.serialPortDWM1001.close()

    def dataValidness(self, data, size):
        """
        Checks if all elements in msg have at least 'size' bytes
        :param: array, int
        :returns: bool
        """
        for element in data:
            if len(element) < size:
                return False
        True

    def getAnchorData(self):
        # Show distances to ranging anchors and the position if location engine is enabled
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LES)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        #time.sleep(0.05) # improves serial reading when stationary i.e. returns non empty anchor_poses

        # Read data
        is_data_valid = False
        n_attempts = 0
        while is_data_valid == False:
            data = self.readSerial()
            is_data_valid = self.dataValidness(data, 25)
            n_attempts += 1
            if n_attempts > 25: # max attempts to read serial, not related with dataValidness
                return []
        # Stop read
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        # Now each element of data array has de following format anchor_id[X,Y,Z]=distance_to_tag
        anchor_poses = []
        for anchor_data in data:
            anchor_id, anchor_data = anchor_data.split("[")
            anchor_id = anchor_id[-4:] # ensure 4 bytes anchor_id
            anchor_pose, anchor_distance = anchor_data.split("]=")
            anchor_poses.append([anchor_id, anchor_pose, anchor_distance])
        return anchor_poses

    def loop(self) :
        """
        Read and publish data
        :param:
        :returns: none
        """
        #Topic 1: PoseWitchCovariance
        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = self.world_frame_id
        pub_pose_with_cov.publish(pwc)

        #Topic 2: IMU
        imu = Imu()
        imu.header.stamp = rospy.get_rostime()
        imu.header.frame_id = self.tag_frame_id
        #imu.orientation =  pypozyx.Quaternion()
        imu.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        #imu.angular_velocity = pypozyx.AngularVelocity()
        imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        #imu.linear_acceleration = pypozyx.LinearAcceleration()
        imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        pub_imu.publish(imu)

        """
        pwc.pose.pose.position = Coordinates()
        pwc.pose.pose.orientation =  pypozyx.Quaternion()
        cov = pypozyx.PositionError()

        status = self.pozyx.doPositioning(pwc.pose.pose.position, self.dimension, self.height, self.algorithm, self.tag_device_id)
        pozyx.getQuaternion(pwc.pose.pose.orientation, self.tag_device_id)
        pozyx.getPositionError(cov, self.tag_device_id)

        cov_row1 =[cov.x, cov.xy, cov.xz, 0, 0, 0]
        cov_row2 =[cov.xy, cov.y, cov.yz, 0, 0, 0]
        cov_row3 =[cov.xz, cov.yz, cov.z, 0, 0, 0]
        cov_row4 =[0, 0, 0, 0, 0, 0]
        cov_row5 =[0, 0, 0, 0, 0, 0]
        cov_row6 =[0, 0, 0, 0, 0, 0]

        pwc.pose.covariance = cov_row1 + cov_row2 + cov_row3 + cov_row4 + cov_row5 + cov_row6
        """
        
        anchor_data = self.getAnchorData()

        if self.dataValidness(anchor_data, 25) == True:
            #Topic 3: Anchors Info
            for i in range(len(anchors)):
                dr = AnchorInfo()
                dr.header.stamp = rospy.get_rostime()
                dr.header.frame_id = self.world_frame_id
                dr.id = anchor_data[i][0]
                x, y, z = map(float, anchor_data[1][1].split(','))
                dr.position.x = x
                dr.position.y = y
                dr.position.z = z

                dr.distance = float(anchor_data[i][2])
                dr.RSS = 0

                if True: # TODO: check if anchor_id is in list of expected anchors to be in the network
                    dr.status = True
                    self.range_error_counts[i] = 0
                else:
                    dr.status = False
                    self.range_error_counts[i] += 1
                    if self.range_error_counts[i] > 9:
                        self.range_error_counts[i] = 0
                        rospy.logerr("Anchor %d (%s) lost", i, dr.id)

                # device_range = DeviceRange()
                # status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
                # dr.distance = (float)(device_range.distance) * 0.001
                # dr.RSS = device_range.RSS

                # if status == POZYX_SUCCESS:
                #     dr.status = True
                #     self.range_error_counts[i] = 0
                # else:
                #     status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
                #     dr.distance = (float)(device_range.distance) * 0.001
                #     dr.RSS = device_range.RSS
                #     if status == POZYX_SUCCESS:
                #         dr.status = True
                #         self.range_error_counts[i] = 0
                #     else:
                #         dr.status = False
                #         self.range_error_counts[i] += 1
                #         if self.range_error_counts[i] > 9:
                #             self.range_error_counts[i] = 0
                #             rospy.logerr("Anchor %d (%s) lost", i, dr.id)

                dr.child_frame_id = anchor_data[i][0]
                pub_anchor_info[i].publish(dr)

    """

    def setup(self):
        self.setAnchorsManual()
        self.printPublishConfigurationResult()

    def setAnchorsManual(self):
        status = self.pozyx.clearDevices()
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor)
        if len(anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors))
        return status

    def printPublishConfigurationResult(self):
        list_size = SingleRegister()
        status = self.pozyx.getDeviceListSize(list_size)
        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list)

        print("Anchors configuration:")
        print("Anchors found: {0}".format(list_size[0]))

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates)
            print("ANCHOR,0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
    """


if __name__ == "__main__":

    rospy.init_node('dwm1001_node')

    # Reading parameters
    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')

    num_anchors = int(rospy.get_param('~num_anchors', 4))
    tag_device_id = (rospy.get_param('~tag_device_id', 'None'))

    algorithm = int(rospy.get_param('~algorithm'))
    dimension = int(rospy.get_param('~dimension'))
    height    = int(rospy.get_param('~height'))
    frequency = int(rospy.get_param('~frequency'))

    world_frame_id = rospy.get_param('~world_frame_id', 'world')
    tag_frame_id = rospy.get_param('~tag_frame_id', 'pozyx_tag')

    do_ranging_attempts = rospy.get_param('~do_ranging_attempts', 1)

    # Creating publishers
    pub_pose_with_cov = rospy.Publisher('~pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pub_imu = rospy.Publisher('~imu', Imu, queue_size=1)

    anchors = []
    for i in range(num_anchors):
        #anchors.append(DeviceCoordinates(anchor_id[i], 1, Coordinates(anchor_coordinates[i][0], anchor_coordinates[i][1], anchor_coordinates[i][2])))
        anchors.append(1)

    pub_anchor_info = []

    for i in range(len(anchors)):
        topic_name = "~anchor_info_" + str(i)
        pub_anchor_info.append(rospy.Publisher(topic_name, AnchorInfo, queue_size=1))

    pub_pose = rospy.Publisher('~pose', PoseStamped , queue_size=1)

    # ROS rate
    rate = rospy.Rate(1000)
    # Starting communication with DWM1001 module
    rdl = ReadyToLocalize(anchors, do_ranging_attempts, world_frame_id, tag_frame_id, tag_device_id, algorithm, dimension, height)
    rdl.initSerial()
    #rdl.setup()
    while not rospy.is_shutdown():
        try:
            rdl.loop()
        except KeyboardInterrupt:
            rdl.handleKeyboardInterrupt()
        rate.sleep()

    rdl.quit()

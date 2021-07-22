#!/usr/bin/env python

""" 
@file: dwm1001_driver_ros.py
@description: ros node to publish distance between tag and anchors
              using UART API from https://www.decawave.com/dwm1001/api/
@author: Esau Ortiz
@date: july 2021
"""

import os, sys
from pickle import FALSE
import rospy, time, serial, random
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from uwb_msgs.msg import AnchorInfo

from dwm1001_apiCommands import DWM1001_API_COMMANDS
from dwm1001_apiCommands import DWMAnchorPosesReq

class ReadyToLocalize(object):

    def __init__(self, anchor_id_list, anchor_coord_list, do_ranging_attempts, world_frame_id, tag_frame_id, tag_device_id, algorithm=None, dimension=None, height=1000):
        """
        Initialize serial port
        """
        self.anchor_id_list = anchor_id_list
        self.anchor_coord_list = anchor_coord_list
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.range_error_counts = [0 for i in range(len(self.anchor_id_list))]
        self.world_frame_id = world_frame_id
        self.tag_frame_id = tag_frame_id
        self.tag_device_id = tag_device_id
        self.do_ranging_attempts = do_ranging_attempts

        # Get some other params
        self.is_location_engine_enabled = bool(rospy.get_param('~location_engine_enable'))
        self.dwm_port = rospy.get_param('~serial_port')
        self.use_network = rospy.get_param('~use_network', False)
        self.network = rospy.get_param('~network_id', "default")
        self.verbose = rospy.get_param('~verbose', False)

        # Empty dictionary to store topics being published
        self.topics = {}
        
    def initSerial(self):
        """
        Initialize port and dwm1001 api
        Parameters
        ----------
        Returns
        ----------
        """

        # Serial port settings
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS,
            timeout = 0.2
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
        Parameters
        ----------
        Returns
        ----------
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
        time.sleep(0.5)

        # set anchor position lecture 
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LES)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

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

    def readSerial(self, command):
        """
        Read serial string and return data as array
        Parameters
        ----------
        command : b
            DWM1001_API_COMMANDS
        Returns
        -------
        array_data : array
            data as array
        """
        try:
            serial_read_line = self.serialPortDWM1001.read_until()
        except:
            return ['']
        array_data = [x.strip() for x in serial_read_line.strip().split(' ')]
        if command in array_data:
            return ['']
        return array_data

    def getDataFromSerial(self, dwm_request, read_attempts = 10):
        """ Tries to read retrieved 'data' with 'expected_size' bytes
        from serial port sending 'command'
        Parameters
        ----------
        command : b (Bytes)
            DWM1001_API_COMMANDS command
        expected_size : int
            number of expected bytes. Should be the minimum expected size
        read_attempts : int
            attempts to read 'data' with at least 'expected_size' bytes
        Returns
        -------
        data : array
            retrieved data
        """
        # Send command
        #self.serialPortDWM1001.write(dwm_request.command)
        #self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        
        # Read data
        is_data_valid = False
        n_attempts = 0
        while is_data_valid == False:
            data = self.readSerial(dwm_request)
            is_data_valid = dwm_request.validness(data)
            n_attempts += 1
            if n_attempts > read_attempts: # max attempts to read serial
                return []
        return data

    def getAnchorsData(self):
        """ Read and formats serial data
        Parameters
        ----------
        Returns
        ----------
        """
        # Show distances to ranging anchors and the position if location engine is enabled
        anchor_data_request = DWMAnchorPosesReq(self.is_location_engine_enabled)
        data = self.getDataFromSerial(anchor_data_request)
        if data == []:
            return [], None
        if self.is_location_engine_enabled == True:
            anchor_data_array, tag_pose = [data[:-2], data[-1]]
            if tag_pose[:3] != 'est':
                tag_pose = None
        else:
            anchor_data_array = data
            tag_pose = None
        # Now each element of anchor_data_array has de following format: anchor_id[X,Y,Z]=distance_to_tag
        anchor_poses = []
        for anchor_data in anchor_data_array:
            anchor_id, anchor_data = anchor_data.split("[")
            anchor_id = anchor_id[-4:] # ensure 4 bytes anchor_id
            anchor_pose, anchor_distance = anchor_data.split("]=")
            anchor_poses.append([anchor_id, anchor_pose, anchor_distance])

        return anchor_poses, tag_pose

    def loop(self, debug = False) :
        """
        Read and publish data
        Parameters
        ----------
        Returns
        ----------
        """
        # read anchor data (always) and estimated tag pose (optional)
        anchor_data_list, tag_pose = self.getAnchorsData()
        anchor_id_list = []
        anchor_coord_list = []
        anchor_distance_list = []
        for anchor_data in anchor_data_list:
            anchor_id_list.append('DW' + anchor_data[0])
            anchor_coord_list.append(anchor_data[1].split(','))
            anchor_distance_list.append(anchor_data[2])
            
        if tag_pose is not None:
            tag_pose = tag_pose[4:]
            tag_pose_x, tag_pose_y, tag_pose_z, _ = tag_pose.split(',')
            # Topic: PoseWitchCovariance
            pwc = PoseWithCovarianceStamped()
            pwc.header.stamp = rospy.get_rostime()
            pwc.header.frame_id = self.world_frame_id
            pwc.pose.pose.position.x = float(tag_pose_x)
            pwc.pose.pose.position.y = float(tag_pose_y)
            pwc.pose.pose.position.z = float(tag_pose_z)
            pub_pose_with_cov.publish(pwc)
            
            # Topic 4: PoseStamped
            ps = PoseStamped()
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.world_frame_id
            ps.pose.position = pwc.pose.pose.position
            ps.pose.orientation =  pwc.pose.pose.orientation
            pub_pose.publish(ps)

        # Topic: Anchors Info
        if anchor_data_list != []:
            if debug:
                print(anchor_id_list)
            for self_anchor_id in self.anchor_id_list:
                dr = AnchorInfo()
                dr.header.stamp = rospy.get_rostime()
                dr.header.frame_id = self.world_frame_id
                dr.id = self_anchor_id
                dr.RSS = 0
                self_idx = self.anchor_id_list.index(dr.id)
                idx = None

                if dr.id in anchor_id_list:
                    idx = anchor_id_list.index(dr.id)
                    dr.status = True
                    self.range_error_counts[self_idx] = 0

                    x, y, z = anchor_coord_list[idx]
                    dr.position.x = float(x)
                    dr.position.y = float(y)
                    dr.position.z = float(z)
                    dr.distance = float(anchor_distance_list[idx])
                    #print('found anchor ' + str(dr.id) + ' with coords (' + str(dr.position.x) + ', ' + str(dr.position.y) + ', ' + str(dr.position.x) + ') and distance ' + str(dr.distance))
                else:
                    dr.status = False
                    self.range_error_counts[self_idx] += 1
                    if self.range_error_counts[self_idx] > 9:
                        self.range_error_counts[self_idx] = 0
                        #rospy.logerr("Anchor %d (%s) lost", i, dr.id)

                dr.child_frame_id = self_anchor_id
                pub_anchor_info[self_idx].publish(dr)

if __name__ == "__main__":

    rospy.init_node('dwm1001_node')

    # Read parameters
    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')

    num_anchors = int(rospy.get_param('~n_anchors', 4))
    tag_device_id = (rospy.get_param('~tag_id', 'None'))

    algorithm = int(rospy.get_param('~algorithm'))
    dimension = int(rospy.get_param('~dimension'))
    height    = int(rospy.get_param('~height'))
    frequency = int(rospy.get_param('~frequency'))

    world_frame_id = rospy.get_param('~world_frame_id', 'world')
    tag_frame_id = rospy.get_param('~tag_frame_id', 'dwm1001_tag')

    do_ranging_attempts = rospy.get_param('~do_ranging_attempts', 1)

    # Read anchors id and pose
    anchors_id = []
    anchors_coord = []
    for i in range(num_anchors):
        anchor_id = rospy.get_param("~anchor" + str(i) + "_id")
        anchor_coord = rospy.get_param("~anchor" + str(i) + "_coordinates")
        anchor_coord = anchor_coord.split(', ')
        anchor_coord = [float(i) for i in anchor_coord]
        anchors_id.append(anchor_id)
        anchors_coord.append(anchor_coord)

    # Creating publishers
    pub_pose_with_cov = rospy.Publisher('~pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pub_pose = rospy.Publisher('~pose', PoseStamped , queue_size=1)
    pub_anchor_info = []

    for i in range(num_anchors):
        topic_name = "~anchor_info_" + str(i)
        pub_anchor_info.append(rospy.Publisher(topic_name, AnchorInfo, queue_size=1))

    # ROS rate
    rate = rospy.Rate(10)
    # Starting communication with DWM1001 module
    rdl = ReadyToLocalize(anchors_id, anchors_coord, do_ranging_attempts, world_frame_id, tag_frame_id, tag_device_id, algorithm, dimension, height)
    rdl.initSerial()
    #rdl.setup()
    while not rospy.is_shutdown():
        try:
            rdl.loop(debug=False)
        except KeyboardInterrupt:
            rdl.handleKeyboardInterrupt()
        rate.sleep()

    rdl.quit()

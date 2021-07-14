#!/usr/bin/env python

""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import os, sys
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
            bytesize = serial.SEVENBITS
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
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
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
        rospy.Rate(1).sleep()
        serialReadLine = self.serialPortDWM1001.read_until()
        if "reset" in serialReadLine:
            rospy.loginfo("succesfully closed ")
            self.serialPortDWM1001.close()

    def loop(self) :
        """
        Read and publish data
        :param:
        :returns: none
        """
        # just read everything from serial port
        serialReadLine = self.serialPortDWM1001.read_until()
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

        #Topic 3: Anchors Info
        for i in range(len(anchors)):
            dr = AnchorInfo()
            dr.header.stamp = rospy.get_rostime()
            dr.header.frame_id = self.world_frame_id
            dr.id = hex(anchors[i].network_id)
            dr.position.x = (float)(anchors[i].pos.x) * 0.001
            dr.position.y = (float)(anchors[i].pos.y) * 0.001
            dr.position.z = (float)(anchors[i].pos.z) * 0.001

            iter_ranging = 0
            while iter_ranging < self.do_ranging_attempts:

                device_range = DeviceRange()
                status = self.pozyx.doRanging(self.anchors[i].network_id, device_range, self.tag_device_id)
                dr.distance = (float)(device_range.distance) * 0.001
                dr.RSS = device_range.RSS

                if status == POZYX_SUCCESS:
                    dr.status = True
                    self.range_error_counts[i] = 0
                    iter_ranging = self.do_ranging_attempts
                else:
                    dr.status = False
                    self.range_error_counts[i] += 1
                    if self.range_error_counts[i] > 9:
                        self.range_error_counts[i] = 0
                        rospy.logerr("Anchor %d (%s) lost", i, dr.id)
                iter_ranging += 1



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

            dr.child_frame_id = "anchor_" + str(i)
            pub_anchor_info[i].publish(dr)
            
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
        
        try:
            self.publishTagPositions(serialReadLine)

        except IndexError:
            rospy.loginfo("Found index error in the network array! DO SOMETHING!")

    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """

        arrayData = [x.strip() for x in serialData.strip().split(',')]

        # If getting a tag position
        if "DIST" in arrayData[0] :

            # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
            number_of_anchors = (len(arrayData) - 7)/6
            rospy.loginfo("Number of anchors: " + str(number_of_anchors))

            for i in range(number_of_anchors) :

                node_id = arrayData[2+6*i]
                first_time = False
                if node_id not in self.topics :
                    first_time = True
                    self.topics[node_id] = rospy.Publisher(
                        '/dwm1001' + 
                        "{}".format("/"+self.network if self.use_network else "") + 
                        '/anchor/' + node_id + 
                        "/position", 
                        PoseStamped, 
                        queue_size=100
                    )
                    self.topics[node_id+"_dist"] = rospy.Publisher(
                        '/dwm1001' + 
                        "{}".format("/"+self.network if self.use_network else "") + 
                        '/tag/' + self.tag_device_id +
                        '/to/anchor/' + node_id +
                        "/distance", 
                        Float64, 
                        queue_size=100
                    )
                try :
                    p = PoseStamped()
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = float(arrayData[4+6*i])
                    p.pose.position.y = float(arrayData[5+6*i])
                    p.pose.position.z = float(arrayData[6+6*i])
                    p.pose.orientation.x = 0.0
                    p.pose.orientation.y = 0.0
                    p.pose.orientation.z = 0.0
                    p.pose.orientation.w = 1.0
                    self.topics[node_id].publish(p)
                except :
                    pass
                try :
                    dist = float(arrayData[7+6*i])
                    self.topics[node_id+"_dist"].publish(dist)
                except :
                    pass

                if self.verbose or first_time :
                    rospy.loginfo("Anchor " + node_id + ": "
                                  + " x: "
                                  + str(p.pose.position.x)
                                  + " y: "
                                  + str(p.pose.position.y)
                                  + " z: "
                                  + str(p.pose.position.z))

            # Now publish the position of the tag itself
            if "POS" in arrayData[-5] :

                # Topic is now a tag with same name as node_id
                first_time = False
                if self.tag_device_id not in self.topics :
                    first_time = True
                    self.topics[self.tag_device_id] = rospy.Publisher('/dwm1001/tag/'+self.tag_device_id+"/position", PoseStamped, queue_size=100)
                p = PoseStamped()
                p.header.stamp = rospy.Time.now()  
                p.pose.position.x = float(arrayData[-4])
                p.pose.position.y = float(arrayData[-3])
                p.pose.position.z = float(arrayData[-2])
                p.pose.orientation.x = 0.0
                p.pose.orientation.y = 0.0
                p.pose.orientation.z = 0.0
                p.pose.orientation.w = 1.0
                self.topics[self.tag_device_id].publish(p)

                if self.verbose or first_time :
                    rospy.loginfo("Tag " + self.tag_device_id + ": "
                                  + " x: "
                                  + str(p.pose.position.x)
                                  + " y: "
                                  + str(p.pose.position.y)
                                  + " z: "
                                  + str(p.pose.position.z))

    """
    def loop(self):

        #Topic 1: PoseWitchCovariance
        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = self.world_frame_id
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

        #Convert from mm to m
        pwc.pose.pose.position.x = pwc.pose.pose.position.x * 0.001
        pwc.pose.pose.position.y = pwc.pose.pose.position.y * 0.001
        pwc.pose.pose.position.z = pwc.pose.pose.position.z * 0.001

        if status == POZYX_SUCCESS:
            pub_pose_with_cov.publish(pwc)

        #Topic 2: IMU
        imu = Imu()
        imu.header.stamp = rospy.get_rostime()
        imu.header.frame_id = self.tag_frame_id
        imu.orientation =  pypozyx.Quaternion()
        imu.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        imu.angular_velocity = pypozyx.AngularVelocity()
        imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        imu.linear_acceleration = pypozyx.LinearAcceleration()
        imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        pozyx.getQuaternion(imu.orientation, self.tag_device_id)
        pozyx.getAngularVelocity_dps(imu.angular_velocity, self.tag_device_id)
        pozyx.getLinearAcceleration_mg(imu.linear_acceleration, self.tag_device_id)

        #Convert from mg to m/s2
        imu.linear_acceleration.x = imu.linear_acceleration.x * 0.0098
        imu.linear_acceleration.y = imu.linear_acceleration.y * 0.0098
        imu.linear_acceleration.z = imu.linear_acceleration.z * 0.0098

        #Convert from Degree/second to rad/s
        imu.angular_velocity.x = imu.angular_velocity.x * 0.01745
        imu.angular_velocity.y = imu.angular_velocity.y * 0.01745
        imu.angular_velocity.z = imu.angular_velocity.z * 0.01745

        pub_imu.publish(imu)

        #Topic 3: Anchors Info
        for i in range(len(anchors)):
            dr = AnchorInfo()
            dr.header.stamp = rospy.get_rostime()
            dr.header.frame_id = self.world_frame_id
            dr.id = hex(anchors[i].network_id)
            dr.position.x = (float)(anchors[i].pos.x) * 0.001
            dr.position.y = (float)(anchors[i].pos.y) * 0.001
            dr.position.z = (float)(anchors[i].pos.z) * 0.001

            iter_ranging = 0
            while iter_ranging < self.do_ranging_attempts:

                device_range = DeviceRange()
                status = self.pozyx.doRanging(self.anchors[i].network_id, device_range, self.tag_device_id)
                dr.distance = (float)(device_range.distance) * 0.001
                dr.RSS = device_range.RSS

                if status == POZYX_SUCCESS:
                    dr.status = True
                    self.range_error_counts[i] = 0
                    iter_ranging = self.do_ranging_attempts
                else:
                    dr.status = False
                    self.range_error_counts[i] += 1
                    if self.range_error_counts[i] > 9:
                        self.range_error_counts[i] = 0
                        rospy.logerr("Anchor %d (%s) lost", i, dr.id)
                iter_ranging += 1



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

            dr.child_frame_id = "anchor_" + str(i)
            pub_anchor_info[i].publish(dr)


        #Topic 4: PoseStamped
        ps = PoseStamped()
        ps.header.stamp = rospy.get_rostime()
        ps.header.frame_id = self.world_frame_id
        ps.pose.position = pwc.pose.pose.position
        ps.pose.orientation =  pwc.pose.pose.orientation

        pub_pose.publish(ps)

        #Topic 5: Pressure
        pr = FluidPressure()
        pr.header.stamp = rospy.get_rostime()
        pr.header.frame_id = self.tag_frame_id
        pressure = pypozyx.Pressure()
        
        pozyx.getPressure_Pa(pressure, self.tag_device_id)
        pr.fluid_pressure = pressure.value
        pr.variance = 0

        pub_pressure.publish(pr)

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
    rate = rospy.Rate(10)
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

#!/usr/bin/env python

"""
@file: dwm1001_localization.py
@description: location engine based on Least Squares-Based Method presented
            in [1] "UWB-Based Self-Localization Strategies: A NovelICP-Based 
            Method and a Comparative Assessmentfor Noisy-Ranges-Prone 
            Environments"
@author: Esau Ortiz
@date: july 2021
"""

import rospy
import numpy as np
from uwb_msgs.msg import AnchorInfo
from geometry_msgs.msg import PoseStamped
from UWBiekf import UWB3D_iekf
import tf

class AnchorSubscriber(object):
    def callback(self, anchor_info):
        self.anchor_info = anchor_info
        self.new_anchor_info = True
    def __init__(self, idx, tag_id):
        self.anchor_info = None
        self.new_anchor_info = False
        rospy.Subscriber("/" + tag_id + "_tag_node/anchor_info_" + str(idx), AnchorInfo, self.callback, queue_size=1)

class OptitrackSubscriber(object):
    def callback(self, pose):
        self.new_pose = True
        old_nsecs = pose.header.stamp.nsecs
        pose.header.stamp.nsecs -= 500000000
        try:
            pose = self.tf_listener.transformPose('world', pose)
        except:
            self.new_pose = False
        pose.header.stamp.nsecs = old_nsecs
        self.pose = pose
    def __init__(self, topic):
        self.pose = None
        self.new_pose = False
        self.tf_listener = tf.TransformListener()


        rospy.Subscriber(topic, PoseStamped, self.callback, queue_size=1)

class LocationEngine(object):
    def __init__(self, world_frame_id, tag0_id, tag1_id, n_anchors, anchors_poses, ekf_kwargs):
        self.id = 0
        self.world_frame_id = world_frame_id
        # set anchor subscribers
        self.tag_coords = []
        self.tag_status = False
        self.anchor_subs_list = []
        for tag_id in [tag0_id, tag1_id]:
            for idx in range(n_anchors):
                self.anchor_subs_list.append(AnchorSubscriber(idx, tag_id))
        # set estimated coordinates pub
        self.estimated_coord_pub = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)
        self.optitrack_in_world = rospy.Publisher("~tag_pose_gt", PoseStamped, queue_size=1)
        self.landmarks = anchor_poses
        # sub to optitrack robot pose
        self.optitrack_sub = OptitrackSubscriber("/optitrack/kobuki_c/pose")

        if ekf_kwargs['using_ekf']:
            initial_pose = np.array([1.1259,3.0572,0.270672,0,0,0]) # manually from optitrack
            #initial_pose = np.array([3.12,1.25,0.270672,0,0,0]) # manually from optitrack
            self.ekf = UWB3D_iekf(ftype = 'EKF', x0 = initial_pose, dt = ekf_kwargs['dt'], std_acc = ekf_kwargs['std_acc'], std_rng = ekf_kwargs['std_rng'], landmarks = anchors_poses)
        else:
            self.ekf = None

    def compute_ranges(self, tag_pose, anchors_poses):
        ranges = []
        for anchor_pose in anchors_poses:
            ranges.append(np.linalg.norm((np.array(tag_pose),np.array(anchor_pose))))
        return ranges

    def computeTagCoords(self, anchor_subs_updated):
        """
        Least Squares-Based Method presented in [1]
        Parameters
        ----------
        anchor_subs_updated: AnchorSubscriber list
            new anchor_info from detected anchors whose
            status is 'True' i.e. anchor has distance to
            tag has been updated
        Returns
        ----------
        tag_coord: (3,) array
            (x, y, z) tag coordinates
        """
        n_anchor_subs_updated = len(anchor_subs_updated)
        # (x,y,z) updated anchors coordinates array
        anchors_coord = np.empty((n_anchor_subs_updated, 3))
        anchors_distances = np.empty((n_anchor_subs_updated,))
        for i, anchor_sub in zip(range(n_anchor_subs_updated), anchor_subs_updated): 
            x = anchor_sub.anchor_info.position.x
            y = anchor_sub.anchor_info.position.y
            z = anchor_sub.anchor_info.position.z
            d = anchor_sub.anchor_info.distance
            anchors_coord[i] = [float(x), float(y), float(z)]
            anchors_distances[i] = d

        """
        # step by step 
        N = n_anchor_subs_updated - 1
        A = np.empty((N, 3))
        for i in range(N):
            for j in range(3): # xyz coords
                A[i][j] = 2 * (anchors_coord[N][j] - anchors_coord[i][j])

        B = np.empty((N,))
        for i in range(N):
            coord_squared_sum_i = 0
            for coord in anchors_coord[i]:
                coord_squared_sum_i -= coord**2
        
            coord_squared_sum_N = 0
            for coord in anchors_coord[N]:
                coord_squared_sum_N += coord**2
        
            B[i] = anchors_distances[i]**2 - anchors_distances[N]**2 + coord_squared_sum_i + coord_squared_sum_N
        """

        # build A matrix
        A = 2 * np.copy(anchors_coord)
        for i in range(A.shape[0] - 1): A[i] = A[-1] - A[i]
        A = A[:-1] # remove last row

        # build B matrix
        B = np.copy(anchors_distances)**2
        B = B[:-1] - B[-1] - np.sum(anchors_coord**2, axis = 1)[:-1] + np.sum(anchors_coord[-1]**2, axis = 0)

        return np.dot(np.linalg.pinv(A), B)

    def loop(self, debug = False):
        # updated anchor subs list
        anchor_subs_updated = []
        ranges = []
        for anchor_sub in self.anchor_subs_list:
            # check is subs have received new anchor info
            # also check if anchor status is True (i.e. anchor found)
            if anchor_sub.new_anchor_info == True and anchor_sub.anchor_info.status == True:
                anchor_subs_updated.append(anchor_sub)
                x = anchor_sub.anchor_info.position.x
                y = anchor_sub.anchor_info.position.y
                z = anchor_sub.anchor_info.position.z
                d = anchor_sub.anchor_info.distance
                ranges.append(d)
                if debug:
                    id = anchor_sub.anchor_info.id
                    print('anchor ' + str(id) + ' with coords (' + str(x) + ', ' + str(y) + ', ' + str(z) + ') and distance ' + str(d))
            else:
                ranges.append(-1.0)

        if self.ekf is not None:
            # tag_coord computed through ekf
            self.ekf.predict()
            self.ekf.update(ranges, niter = 100)
            tag_coord = self.ekf.x

        if self.optitrack_sub.new_pose:
            x = self.optitrack_sub.pose.pose.position.x
            y = self.optitrack_sub.pose.pose.position.y
            z = self.optitrack_sub.pose.pose.position.z
            #gt_ranges = self.compute_ranges(np.array([x,y,z]), self.landmarks)
            now = rospy.get_rostime()
            #np.savetxt('/media/esau/hdd_at_ubuntu/bag_files/tmp/' + str(self.id) + '_gt_pose.txt', np.array((x,y,z)))
            #np.savetxt('/media/esau/hdd_at_ubuntu/bag_files/tmp/' + str(self.id) + '_time_stamps.txt', np.array([now.secs, now.nsecs]))
            #np.savetxt('/media/esau/hdd_at_ubuntu/bag_files/tmp/' + str(self.id) + '_ranges.txt', np.array([ranges]))
            self.id +=1
            self.optitrack_in_world.publish(self.optitrack_sub.pose)
            self.optitrack_sub.new_pose = False

        if len(anchor_subs_updated) >= 4:
            if self.ekf is None:
                tag_coord = self.computeTagCoords(anchor_subs_updated)
            ps = PoseStamped()
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.world_frame_id
            ps.pose.position.x = tag_coord[0]
            ps.pose.position.y = tag_coord[1]
            ps.pose.position.z = tag_coord[2]
            self.estimated_coord_pub.publish(ps)

            if debug: 
                print(str(len(anchor_subs_updated)) + ' anchor-tag distances have been received, computing tag coords ...')
                print(tag_coord)
                
        # discard msgs if they have not arrived during one rate.sleep()
        #for anchor_sub in self.anchor_subs_list: anchor_sub.new_anchor_info = False
        
        if debug: print('\n')

def stop_node(event):
    rospy.signal_shutdown("Shutting down localization...")

if __name__ == '__main__':

    rospy.init_node('dwm1001_localization')
    # ROS rate
    rate = rospy.Rate(12.5)

    # read how many anchors are in the network
    n_anchors = int(rospy.get_param('~n_anchors'))
    world_frame_id = str(rospy.get_param('~world_frame_id'))
    tag0_id = rospy.get_param('~tag0_id')
    tag1_id = rospy.get_param('~tag1_id')
    # ekf params
    using_ekf = rospy.get_param('~using_ekf')
    std_acc = float(rospy.get_param('~std_acc'))
    std_rng = float(rospy.get_param('~std_rng'))
    dt = float(rospy.get_param('~dt'))
    ekf_kwargs = {'using_ekf' : using_ekf, 'std_acc' : std_acc, 'std_rng' : std_rng, 'dt' : dt}

    anchor_poses = np.empty((n_anchors, 3))
    for i in range(n_anchors):
        anchor_poses[i] = rospy.get_param('~anchor' + str(i) + '_coordinates')

    # location engine object
    location_engine = LocationEngine(world_frame_id, tag0_id, tag1_id, n_anchors, anchor_poses, ekf_kwargs)

    # if 0 then duration until KeyboardInterrupt
    if int(rospy.get_param('~duration')) != 0:
        rospy.Timer(rospy.Duration.from_sec(float(rospy.get_param('~duration'))), stop_node)
    
    while not rospy.is_shutdown():
        try:
            location_engine.loop(debug=True)
        except KeyboardInterrupt:
            pass
            # location_engine.handleKeyboardInterrupt()
        rate.sleep()

#!/usr/bin/env python

"""
@file: dwm1001_location.py
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
from geometry_msgs.msg import Pose


class AnchorSubscriber(object):
    def callback(self, anchor_info):
        self.anchor_info = anchor_info
        self.new_anchor_info = True
    def __init__(self, idx):
        self.anchor_info = None
        self.new_anchor_info = False
        rospy.Subscriber("dwm1001_node/anchor_info_" + str(idx), AnchorInfo, self.callback)

class LocationEngine(object):
    def __init__(self, n_anchors):
        # set anchor subscribers
        self.tag_coords = []
        self.tag_status = False
        self.anchor_subs_list = []
        for idx in range(n_anchors):
            self.anchor_subs_list.append(AnchorSubscriber(idx))
        # set estimated coordinates pub
        self.estimated_coord_pub = rospy.Publisher("dwm1001_location", Pose, queue_size=10)

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

        # build A matrix
        A = np.copy(anchors_coord)
        for i in range(A.shape[0] - 1): A[i] = A[-1] - A[i]
        A = A[:-1] # remove last row

        # build B matrix
        B = np.copy(anchors_distances)
        for i in range(B.shape[0] - 1):
            # ri^2  - rN^2     - xi^2 - yi^2 - zi^2                + xN^2 + yN^2 + zN^2
            B[i] += - B[-1]**2 - (anchors_coord[i]**2).sum(axis=0) + (anchors_coord[-1]**2).sum(axis=0)
        B = B[:-1] # remove last element

        return np.dot(np.linalg.pinv(A), B)

    def loop(self, debug = False):
        # updated anchor subs list
        anchor_subs_updated = []
        for anchor_sub in self.anchor_subs_list:
            # check is subs have received new anchor info
            # also check if anchor status is True (i.e. anchor found)
            if anchor_sub.new_anchor_info == True and anchor_sub.anchor_info.status == True:
                anchor_subs_updated.append(anchor_sub)
                x = anchor_sub.anchor_info.position.x
                y = anchor_sub.anchor_info.position.y
                z = anchor_sub.anchor_info.position.z
                d = anchor_sub.anchor_info.distance
                if debug:
                    id = anchor_sub.anchor_info.id
                    print('anchor ' + str(id) + ' with coords (' + str(x) + ', ' + str(y) + ', ' + str(z) + ') and distance ' + str(d))

        if len(anchor_subs_updated) >= 4:
            tag_coord = self.computeTagCoords(anchor_subs_updated)
            if debug: 
                print('Four anchor-tag distances have been received, computing tag coords ...')
                print(tag_coord)
                
        # discard msgs if they have not arrived during one rate.sleep()
        for anchor_sub in self.anchor_subs_list:
            anchor_sub.new_anchor_info = False
        
        if debug: print('\n')

if __name__ == '__main__':

    rospy.init_node('dwm1001_ls_location')

    # ROS rate
    rate = rospy.Rate(5)

    # read how many anchors are in the network
    n_anchors = 5

    # location engine object
    location_engine = LocationEngine(n_anchors)

    while not rospy.is_shutdown():
        try:
            location_engine.loop(debug=True)
        except KeyboardInterrupt:
            pass
            # location_engine.handleKeyboardInterrupt()
        rate.sleep()

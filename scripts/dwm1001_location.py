#!/usr/bin/env python

"""
@file: dwm1001_location.py
@description: location engine based on Least Squares-Based Method presented
            in "UWB-Based Self-Localization Strategies: A NovelICP-Based 
            Method and a Comparative Assessmentfor Noisy-Ranges-Prone 
            Environments"
@author: Esau Ortiz
@date: july 2021
"""

import rospy
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
        self.anchor_subs_list = []
        for idx in range(n_anchors):
            self.anchor_subs_list.append(AnchorSubscriber(idx))
        # set estimated coordinates pub
        self.estimated_coord_pub = rospy.Publisher("dwm1001_location", Pose, queue_size=10)

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
                id = anchor_sub.anchor_info.id
                #print('anchor ' + str(id) + ' with coords (' + str(x) + ', ' + str(y) + ', ' + str(z) + ') and distance ' + str(d))
        # if 4 or more anchor subs has new coordinates compute estimated tag coords
        if len(anchor_subs_updated) >= 4:
            True
            # compute estimated tag coord
        # discard msgs
        for anchor_sub in self.anchor_subs_list:
            anchor_sub.new_anchor_info = False
        print('\n')

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
            location_engine.loop(debug=False)
        except KeyboardInterrupt:
            pass
            # location_engine.handleKeyboardInterrupt()
        rate.sleep()

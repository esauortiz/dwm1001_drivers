#!/usr/bin/env python

""" 
@file: autocalibration_sample_uart.py
@description: python module to configure anchor as tag, retrieve ranges and configure module as anchor back
              using UART API from https://www.decawave.com/dwm1001/api/
@author: Esau Ortiz
@date: october 2021
@usage: python autocalibration_sample_uart.py <n_samples> <nodes_configuration_label> <n_samples>

        # where 
                <nodes_configuration_label> is a yaml file which includes nets, 
                tag ids, anchor ids and anchor coords
                <n_samples> samples to save when retrieving ranges
                <module> is the module id with DW1234 format
"""

from dwm1001_apiCommands import DWM1001_UART_API
from dwm1001_apiCommands import DWMRangingReq
from pathlib import Path
import time, serial, yaml, sys
import numpy as np

class ReadyToCalibrate(DWM1001_UART_API):

    def __init__(self, verbose = False):
        self.verbose = verbose

    def getAnchorsData(self, is_location_engine_enabled = False, verbose = False):
        """ Read and formats serial data
        Parameters
        ----------
        Returns
        ----------
        """
        # Show distances to ranging anchors and the position if location engine is enabled
        ranging_request = DWMRangingReq(is_location_engine_enabled)
        data = self.getDataFromSerial(ranging_request, verbose)
        if data == []:
            return None
        if is_location_engine_enabled == True:
            anchor_data_array, tag_pose = [data[:-2], data[-1]]
            if tag_pose[:3] != 'est':
                tag_pose = None
        else:
            anchor_data_array = data
            tag_pose = None
        # Now each element of anchor_data_array has de following format: anchor_id[X,Y,Z]=distance_to_tag
        ranging_data = {}
        for anchor_data in anchor_data_array:
            anchor_id, anchor_data = anchor_data.split("[")
            anchor_id = anchor_id[-4:] # ensure 4 bytes anchor_id
            anchor_pose, anchor_distance = anchor_data.split("]=")
            ranging_data[anchor_id] = float(anchor_distance)

        return ranging_data

def readYaml(file):
    with open(file, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

def main():
    # initiator enabler
    initiator_enabler = int(sys.argv[3])
    # target module from which retrieve ranges
    target_dwm_module = sys.argv[4]
    # load nodes configuration label
    try: nodes_configuration_label = sys.argv[2]
    except: nodes_configuration_label = 'default'
    # load nodes configuration label
    try: n_samples = int(sys.argv[1])
    except: n_samples = 10

    # load anchors cfg
    current_path = Path(__file__).parent.resolve()
    dwm1001_drivers_path = str(current_path.parent)
    nodes_cfg = readYaml(dwm1001_drivers_path + "/params/nodes_cfg/" + nodes_configuration_label + ".yaml")

    # set some node variables
    n_networks = nodes_cfg['n_networks']
    network_id_list = []
    anchor_id_list = [] # single level list
    anchor_id_list_by_network = [] # grouped by networks
    for i in range(n_networks):
        network_cfg = nodes_cfg['network' + str(i)]
        network_id_list.append(network_cfg['network_id'])
        n_anchors = network_cfg['n_anchors']
        anchors_in_network_list = [network_cfg['anchor' + str(i) + '_id'] for i in range(n_anchors)]
        anchor_id_list_by_network.append(anchors_in_network_list)
        anchor_id_list += anchors_in_network_list

    # matrix to save ranging samples
    n_total_anchors = len(anchor_id_list)
    ranging_data = -np.ones((n_samples, n_total_anchors), dtype = float)

    # Read parameters
    serial_port = '/dev/' + target_dwm_module

    # Starting communication with DWM1001 module
    rtc = ReadyToCalibrate()
    rtc.initSerial(serial_port)

    # set module as tag
    meas_mode = 0   # not used
    stnry_en = 1    # stationary mode enabled
    low_pwr = 0     # low power mode disabled
    loc_en = 0      # location engine disabled
    enc = 0         # encription disabled
    leds = 1        # leds enabled
    ble = 1         # ble enabled
    uwb = 2         # UWB active mode
    fw_upd = 0      # firmware update disabled
    rtc.acts([meas_mode, stnry_en, low_pwr, loc_en, enc, leds, ble, uwb, fw_upd])

    # set ranging frequency to max (10Hz)
    rtc.aurs([1,1])

    for network_id in network_id_list:
        # set module network
        rtc.nis(network_id)
        # reinitialize serial to apply changes (actually resets module)
        rtc.quit()
        rtc.initSerial(serial_port)
        # set ranging mode
        rtc.les()

        # read ranges
        for sample_idx in range(n_samples):
            print('Retrieving ranges')
            location_data = rtc.getAnchorsData()
            print(location_data)
            if location_data is not None:
                for anchor in location_data:
                    col_idx = anchor_id_list.index('DW' + anchor)
                    row_idx = sample_idx
                    ranging_data[row_idx, col_idx] = location_data[anchor]
            time.sleep(0.1)
    
    np.savetxt(target_dwm_module + '_ranging_data.txt', ranging_data)
    
    # set module as anchor
    initiator = initiator_enabler   # initiator enabled
    bridge_en = 0   # bridge mode disabled
    enc_en = 0      # encryption disabled
    leds = 1        # leds disabled
    ble = 1         # ble enabled
    uwb = 2         # UWB active mode
    fw_upd = 0      # firmware update disabled
    rtc.acas([initiator, bridge_en, enc_en, leds, ble, uwb, fw_upd])

    # set module network
    network_id = None
    for i in range(n_networks):
        if target_dwm_module in anchor_id_list_by_network[i]:
            network_id = network_id_list[i]    
    rtc.nis(network_id)

    rtc.quit()
    rtc.initSerial(serial_port)    
    rtc.quit()

if __name__ == "__main__":
    main()
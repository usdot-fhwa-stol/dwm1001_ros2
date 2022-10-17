#!/usr/bin/env python3

# Copyright 2022 Volpe National Transportation Systems Center.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# this file contains a list of tools to be used to simplify smarter working with Serial

# import serial.tools.list_ports as lports
from serial.tools import list_ports as lports
# import serial.tools.list_ports.ListPortInfo

# return com-port when given a device name (e.g., 'CP2102')
def get_port(str_device_name):
    active_ports = list(lports.comports())
    for port in active_ports:
        # return port if device is found
        if str_device_name in port[1]:
            device_port = port[0]
            return device_port
            break
    # return None if device is not found as an active port
    return None

# returns com-port when given a device serial number (e.g., 000760115296)
def get_port_from_sn(str_sn):
    active_ports = list(lports.comports())
    for port in active_ports:
        # return port if device is found
        if str_sn in str(port[2]):
            device_port = port[0]
            return device_port
            break
    # return None if device SN is not found as an active port
    return None

# get more info from active ports
def _get_more_info():
    print("test")
    active_ports = list(lports.ListPortInfo())
    for port in active_ports:
        print('a')

# get list of all active comports and their device names
def _get_all_active_comports():
    active_ports = list(lports.comports())
    for port in active_ports:
        print("Device Name: %s \nPort: %s\n"%(port[1], port[0]))
        print(port[2])


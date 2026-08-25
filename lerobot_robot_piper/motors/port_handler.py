#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 WeGo-Robotics Inc. EDU team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Absorbed from the wego_piper package (PyPI: wego-piper) so the plugin has
# no dependency on it.

import time

from piper_sdk import C_PiperInterface_V2

DEFAULT_BAUDRATE = 1000000


class PortHandler(object):
    def __init__(self):
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_open = False
        self.is_using = False
        self.port_name = ""
        self.ser: C_PiperInterface_V2 = None

    def openPort(self) -> bool:
        self.ser.ConnectPort()
        self.is_open = self.ser.get_connect_status()
        return self.is_open

    def closePort(self):
        self.ser.DisconnectPort()
        self.is_open = self.ser.get_connect_status()

    def clearPort(self):
        pass

    def setPortName(self, port_name):
        pass

    def getPortName(self):
        return self.ser.GetCanName()

    def setBaudRate(self, baudrate):
        pass

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        return -1

    def readPort(self, length):
        raise NotImplementedError()

    def writePort(self, packet):
        raise NotImplementedError()

    def setPacketTimeout(self, packet_length):
        pass

    def setPacketTimeoutMillis(self, msec):
        pass

    def isPacketTimeout(self):
        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since

    def setupPort(self, controller: C_PiperInterface_V2):
        if self.ser != None:
            self.closePort()

        self.port_name = controller.GetCanName()
        self.ser = controller

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def getCFlagBaud(self, baudrate):
        raise NotImplementedError()

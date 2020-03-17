from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import time
import math
import logging
import threading

import warnings
warnings.simplefilter("ignore", DeprecationWarning)



class VehicleClient:

    mutex = threading.Lock()

    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')

    # -----------------------------------  Common vehicle APIs ---------------------------------------------
    def reset(self):
        self.client.call('reset')

    def ping(self):
        return self.client.call('ping')

    def confirmConnection(self):
        if self.ping():
            print("Connected!")
        else:
             print("Ping returned false!")

    # basic flight control
    def enableApiControl(self, is_enabled):
        return self.client.call('enableApiControl', is_enabled)

    def isApiControlEnabled(self):
        return self.client.call('isApiControlEnabled')

    def armDisarm(self, arm):
        return self.client.call('armDisarm', arm)

    # init_connect
    def connection(self):
        self.confirmConnection()
        self.enableApiControl(True)
        self.armDisarm(True)

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImages(self, requests):
        VehicleClient.mutex.acquire()
        responses_raw = self.client.call('simGetImages', requests)
        tmp = [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]
        VehicleClient.mutex.release()
        return tmp

    # sensor APIs
    def getImuData(self):
        VehicleClient.mutex.acquire()
        tmp = ImuData.from_msgpack(self.client.call('getImudata'))
        VehicleClient.mutex.release()
        return tmp

    def getBarometerData(self):
        VehicleClient.mutex.acquire()
        tmp = BarometerData.from_msgpack(self.client.call('getBarometerdata'))
        VehicleClient.mutex.release()
        return tmp

    def getMagnetometerData(self):
        VehicleClient.mutex.acquire()
        tmp = MagnetometerData.from_msgpack(self.client.call('getMagnetometerdata'))
        VehicleClient.mutex.release()
        return tmp

    # control APIs
    def takeoff(self, max_wait_seconds = 15):
        VehicleClient.mutex.acquire()
        tmp = self.client.call('takeoff', max_wait_seconds)
        VehicleClient.mutex.release()
        return tmp

    def land(self, max_wait_seconds = 60):
        VehicleClient.mutex.acquire()
        tmp = self.client.call('land', max_wait_seconds)
        VehicleClient.mutex.release()
        return tmp

    def hover(self):
        VehicleClient.mutex.acquire()
        tmp = self.client.call('hover')
        VehicleClient.mutex.release()
        return tmp

    def moveByAngleThrottle(self, pitch, roll, throttle, yaw_rate, duration):
        VehicleClient.mutex.acquire()
        tmp = self.client.call('moveByAngleThrottle', pitch, roll, throttle, yaw_rate, duration)
        VehicleClient.mutex.release()
        return tmp
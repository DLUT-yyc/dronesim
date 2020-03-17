from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import time
import math
import logging
import numpy as np
import cv2
import pickle

import warnings
warnings.simplefilter("ignore", DeprecationWarning)

class VehicleImage:
    def __init__(self,client):
        self.client = client

    def getFrontSense(self):
        responses = self.client.simGetImages([ImageRequest(0, ImageType.Scene,False,False)])
        response = responses[0]
        rawImage = np.fromstring(response.image_data_uint8,dtype=np.uint8)
        # rawImage = rawImage.reshape(response.height, response.width, 4)
        rawImage = rawImage.reshape(response.height, response.width, 3)
        # rawImage=cv2.cvtColor(rawImage, cv2.COLOR_BGR2RGB)
        # rawImage = rawImage[:,:,0:3]
        return rawImage

    def getVerticalSense(self):
        responses = self.client.simGetImages([ImageRequest(3, ImageType.Scene,False,False)])
        response = responses[0]
        rawImage = np.fromstring(response.image_data_uint8,dtype=np.uint8)
        # rawImage = rawImage.reshape(response.height, response.width, 4)
        rawImage = rawImage.reshape(response.height, response.width, 3)
        # rawImage=cv2.cvtColor(rawImage, cv2.COLOR_BGR2RGB)
        # rawImage = rawImage[:,:,0:3]
        return rawImage

    def getDepthImage(self):
        responses = self.client.simGetImages([ImageRequest(0, ImageType.DepthPerspective,True,False)])
        response = responses[0]
        rawImage = np.array(response.image_data_float)     #'height': 480,'  'width': 640  image_data_uint8': b'\x00',
        #rawImage = np.array(response.image_data_float)       # (307200,) 307200 float64
        rawImage = rawImage.reshape(response.height, response.width)  #(480, 640)  307200   float64
        return rawImage
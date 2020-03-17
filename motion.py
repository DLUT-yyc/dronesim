import sys
import time
import math
import msvcrt
import threading
import numpy as np

class PID:

    INIT_VALUE=0.5869 * 10000           #抵消重力的油门常数
    MAX_VALUE=INIT_VALUE + 0.4*10000
    MIN_VALUE=INIT_VALUE - 0.4*10000

    def __init__(self,kp=40,ki=0.002,kd=500):
        self.dest=0
        self.Kp=kp
        self.Ki=ki
        self.kd=kd
        self.preerror=0
        self.prederror=0
        self.out=PID.INIT_VALUE

    def reset(self):
        self.preerror = 0
        self.prederror = 0
        self.out=PID.INIT_VALUE

    def setdest(self,dest):
        self.dest=dest
    
    def update(self,feedback):
        error=self.dest-feedback
        derror=error-self.preerror
        dderror=derror-self.prederror
        self.preerror=error
        self.prederror=derror
        
        deltu=self.Kp*derror+self.Ki*error+self.kd*dderror
        self.out+= deltu

        if self.out > PID.MAX_VALUE: self.out=PID.MAX_VALUE
        elif self.out < PID.MIN_VALUE: self.out=PID.MIN_VALUE
        
        return self.out/10000


class VehicleMotion(threading.Thread):

    # THROTTLE_CONSTANT_GRAVITY=0.5869 #抵消重力的油门常数
    THROTTLE_CONSTANT_GRAVITY=0.5952 #抵消重力的油门常数
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client=client
        self.pid=PID()
        self.pitch=0.0
        self.roll=0.0
        self.yaw_rate=0.0
        self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY
        self.thread_stop = False
        self.moving_flag=False
        self.closeloop_flag=False
        self.altitude=0.0
        self.init_t=time.time()

    def getFlyTime(self):
        now_t=time.time()
        return round(now_t-self.init_t,2)
    
    def takeoff(self):
        print(str(self.getFlyTime())+"s: taking off...")
        self.client.takeoff()
        print(str(self.getFlyTime())+"s: dlut-uav is ready! Go!")

    def run(self): 
        while True:
            if self.thread_stop: break
            if self.moving_flag:
                if self.closeloop_flag:
                    self.altitude = self.client.getBarometerData().altitude#.altitude
                    self.throttle = self.pid.update(self.altitude)
                    self.client.moveByAngleThrottleAsync(self.pitch, self.roll,self.throttle/math.cos(self.pitch + self.roll), self.yaw_rate, 2)
                    time.sleep(0.4)
                else:  
                    self.client.moveByAngleThrottleAsync(self.pitch, self.roll, self.throttle, self.yaw_rate, 0.5)
            time.sleep(0.1)

    def endThread(self):
        self.thread_stop = True

    def getMotionState(self):
        s = f"[Motion]  MovingState:{self.moving_flag}  CloseloopState:{self.closeloop_flag}  Pitch:{self.pitch}  Roll:{self.roll}  Yaw_rate:{self.yaw_rate}  Throttle:{self.throttle} DestHeight:{self.getDestHeight()}  Height:{self.getHeight()}"
        return s

    def getHeight(self):
        return round(self.client.getBarometerData().altitude, 2)

    def getDestHeight(self):
        return self.pid.dest

    def flyModeSwitch(self, mode):
        if mode=="hover":
            self.moving_flag = False; self.closeloop_flag = False
        elif mode=="openloop":
            self.moving_flag = True; self.closeloop_flag = False
        elif mode=="closeloop":
            self.pid.reset()
            self.pid.setdest(self.getHeight())
            self.moving_flag = True; self.closeloop_flag = True

    def flyCmd(self, cmd, speed="slow"):
        if cmd=="forward":
            if speed=="slow": self.pitch=-0.01
            elif speed=="fast": self.pitch=-0.05
            else: self.pitch=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.pitch)
            self.roll=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="backward":
            if speed=="slow": self.pitch=0.01
            elif speed=="fast": self.pitch=0.05
            else: self.pitch=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.pitch)
            self.roll=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="moveleft":
            if speed=="slow": self.roll=-0.01
            elif speed=="fast": self.roll=-0.05
            else: self.roll=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.roll)
            self.pitch=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="moveforwardleft":
            if speed=="slow": self.pitch=-0.01; self.roll=-0.05
            else: self.pitch=0.0; self.roll=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.pitch+self.roll)
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="moveright":
            if speed=="slow": self.roll=0.01
            elif speed=="fast": self.roll=0.05
            else: self.roll=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.roll)
            self.pitch=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="moveforwardright":
            if speed=="slow": self.pitch=-0.01; self.roll=0.05
            else: self.pitch=0.0; self.roll=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY/math.cos(self.pitch + self.roll)
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="turnleft":
            if speed=="slow": self.yaw_rate=-0.1
            elif speed=="fast": self.yaw_rate=-0.3
            else: self.yaw_rate=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY
            self.pitch=0.0
            self.roll=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="turnright":
            if speed=="slow": self.yaw_rate=0.1
            elif speed=="fast": self.yaw_rate=0.3
            else: self.yaw_rate=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY
            self.pitch=0.0
            self.roll=0.0
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="up":
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            if speed=="slow": self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY+0.005
            elif speed=="fast": self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY+0.02
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="down":
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            if speed=="slow": self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY-0.005
            elif speed=="fast": self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY-0.02
            self.moving_flag = True; self.closeloop_flag = False
        elif cmd=="pause":
            self.moving_flag=True; self.closeloop_flag=False
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY
        elif cmd=="stop":
            self.moving_flag=False; self.closeloop_flag=False
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY

    def flyToHeight(self,dest_height,delay=10):
        self.pid.setdest(dest_height)
        self.moving_flag = True; self.closeloop_flag = True
        count=0
        while True:
            time.sleep(1)
            count=count+1
            if count==delay: 
                break

    def flyAsHeight(self,dest_height):
        self.pid.setdest(dest_height)
        self.moving_flag = True; self.closeloop_flag = True

    def flyCmd2(self, cmd, speed="slow"):
        if cmd=="forward":
            if speed=="slow": self.pitch=-0.01
            elif speed=="fast": self.pitch=-0.05
            elif speed=="scan_mode": self.pitch=-0.025
            else: self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="backward":
            if speed=="slow": self.pitch=0.01
            elif speed=="fast": self.pitch=0.05
            else: self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="moveleft":
            if speed=="slow": self.roll=-0.01
            elif speed=="fast": self.roll=-0.05
            else: self.roll=0.0
            self.pitch=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="moveright":
            if speed=="slow": self.roll=0.01
            elif speed=="fast": self.roll=0.05
            else: self.roll=0.0
            self.pitch=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="turnleft":
            if speed=="slow": self.yaw_rate=-0.1
            elif speed=="fast": self.yaw_rate=-0.3
            else: self.yaw_rate=0.0
            self.pitch=0.0
            self.roll=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="turnright":
            if speed=="slow": self.yaw_rate=0.1
            elif speed=="fast": self.yaw_rate=0.3
            else: self.yaw_rate=0.0
            self.pitch=0.0
            self.roll=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="pause":
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.moving_flag = True; self.closeloop_flag = True
        elif cmd=="stop":
            self.moving_flag=False; self.closeloop_flag=False
            self.client.hoverAsync()
            self.pitch=0.0
            self.roll=0.0
            self.yaw_rate=0.0
            self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY

    def testGravityConstant(self):
        while True:
            key = msvcrt.getch()
            if key==b'q':
                self.endThread()
                break
            if key==b'c':
                self.pitch=0.0
                self.roll=0.0
                self.yaw_rate=0.0
                self.throttle=self.throttle+0.0001
                self.moving_flag=True
            if key==b'v':
                self.pitch=0.0
                self.roll=0.0
                self.yaw_rate=0.0
                self.throttle=self.throttle-0.0001
                self.moving_flag=True
            if key==b'p':
                self.moving_flag=False
                self.pitch=0.0
                self.roll=0.0
                self.yaw_rate=0.0
                self.throttle=VehicleMotion.THROTTLE_CONSTANT_GRAVITY
            print("throttle="+str(self.throttle)+",height="+str(self.altitude))
    
    def keyControlTest(self):
        key = msvcrt.getch()
        if key==b'Q' or key==b'q': return "quit"
        if key==b'W': self.flyCmd("forward","fast")
        if key==b'w': self.flyCmd("forward","slow")
        if key==b'S': self.flyCmd("backward","fast")
        if key==b's': self.flyCmd("backward","slow")
        if key==b'A': self.flyCmd("moveleft","fast")
        if key==b'a': self.flyCmd("moveleft","slow")
        if key==b'D': self.flyCmd("moveright","fast")
        if key==b'd': self.flyCmd("moveright","slow")
        if key==b'Z': self.flyCmd("turnleft","fast")
        if key==b'z': self.flyCmd("turnleft","slow")
        if key==b'X': self.flyCmd("turnright","fast")
        if key==b'x': self.flyCmd("turnright","slow")
        if key==b'C': self.flyCmd("up","fast")
        if key==b'c': self.flyCmd("up","slow")
        if key==b'V': self.flyCmd("down","fast")
        if key==b'v': self.flyCmd("down","slow")
        if key==b'O' or key==b'o': self.flyCmd("pause")
        if key==b'P' or key==b'p': self.flyCmd("stop")
        print(self.getMotionState())
        return key

    def keyControlTest2(self):
        key = msvcrt.getch()
        if key==b'Q' or key==b'q': return "quit"
        if key==b'W': self.flyCmd2("forward","fast")
        if key==b'w': self.flyCmd2("forward","slow")
        if key==b'S': self.flyCmd2("backward","fast")
        if key==b's': self.flyCmd2("backward","slow")
        if key==b'A': self.flyCmd2("moveleft","fast")
        if key==b'a': self.flyCmd2("moveleft","slow")
        if key==b'D': self.flyCmd2("moveright","fast")
        if key==b'd': self.flyCmd2("moveright","slow")
        if key==b'Z': self.flyCmd2("turnleft","fast")
        if key==b'z': self.flyCmd2("turnleft","slow")
        if key==b'X': self.flyCmd2("turnright","fast")
        if key==b'x': self.flyCmd2("turnright","slow")
        if key==b'C': self.flyAsHeight(self.getDestHeight()+3)
        if key==b'c': self.flyAsHeight(self.getDestHeight()+1)
        if key==b'V': self.flyAsHeight(self.getDestHeight()-3)
        if key==b'v': self.flyAsHeight(self.getDestHeight()-1)
        if key==b'O' or key==b'o': self.flyCmd2("pause")
        if key==b'P' or key==b'p': self.flyCmd2("stop")
        print(self.getMotionState())
        return key


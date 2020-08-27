import smbus
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import sys
import pandas as pd
import RPi.GPIO as GPIO
import concurrent.futures
from threading import Thread

#Program utama dari ROBOT.

#======================PARAMETER: SETUP PINS=======================>> [1]
#Pin terpakai: 4,18,17,22,23,24,25,27,5,12,6,13,19,26
#Set penomeran pin
GPIO.setmode(GPIO.BCM)
#Setup Pin PWM Motor Driver 1 (mengatur kecepatan)
p1PWM1 = 22   #kiri depan
p1PWM2 = 25   #kanan depan
#Setup Pin kontrol maju mundur (mengatur maju/mundur)
p1RODA1A = 17 #kiri depan
p1RODA1B = 27 #kiri depan
p1RODA2A = 23 #kanan depan
p1RODA2B = 24 #kanan depan
#Setup Pin PWM Motor Driver 2
p2PWM1 = 5    #kiribelakang
p2PWM2 = 12   #kanan belakang
#Setup Pin kontrol maju mundur
p2RODA1A = 6  #kiri belakang
p2RODA1B = 13 #kiri belakang
p2RODA2A = 19 #kanan belakang
p2RODA2B = 26 #kanan belakang
#Setup Pin Sonar
TRIG = 4
ECHO = 18
#Setup Pin PWM
GPIO.setup(p1PWM1, GPIO.OUT)
GPIO.setup(p1PWM2, GPIO.OUT)
GPIO.setup(p2PWM1, GPIO.OUT)
GPIO.setup(p2PWM2, GPIO.OUT)
#Setup Pin kontrol maju mundur
GPIO.setup(p1RODA1A, GPIO.OUT)
GPIO.setup(p1RODA1B, GPIO.OUT)
GPIO.setup(p1RODA2A, GPIO.OUT)
GPIO.setup(p1RODA2B, GPIO.OUT)
GPIO.setup(p2RODA1A, GPIO.OUT)
GPIO.setup(p2RODA1B, GPIO.OUT)
GPIO.setup(p2RODA2A, GPIO.OUT)
GPIO.setup(p2RODA2B, GPIO.OUT)
#Setup Pin Sonar
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
#Set PWM
pwm1 = GPIO.PWM(p1PWM1,50)
pwm2 = GPIO.PWM(p1PWM2,50)
pwm3 = GPIO.PWM(p2PWM1,50)
pwm4 = GPIO.PWM(p2PWM2,50)
#======================PARAMETER: SETUP PINS=======================<<

#======================FUNCTION: SET MAJU / MUNDUR=======================>>[2] #note: roda kanan belakang down
def setmaju():
    #Maju SET 1 roda kiri
    GPIO.output(p1RODA1A, GPIO.HIGH)
    GPIO.output(p1RODA1B, GPIO.LOW)
    GPIO.output(p2RODA1A, GPIO.HIGH)
    GPIO.output(p2RODA1B, GPIO.LOW)
    #Maju SET 2 roda kanan
    GPIO.output(p1RODA2A, GPIO.HIGH)
    GPIO.output(p1RODA2B, GPIO.LOW)
    GPIO.output(p2RODA2A, GPIO.HIGH)
    GPIO.output(p2RODA2B, GPIO.LOW)

def setmundur():
    #Maju SET 1
    GPIO.output(p1RODA1A, GPIO.LOW)
    GPIO.output(p1RODA1B, GPIO.HIGH)
    GPIO.output(p2RODA1A, GPIO.LOW)
    GPIO.output(p2RODA1B, GPIO.HIGH)
    #Maju SET 2
    GPIO.output(p1RODA2A, GPIO.LOW)
    GPIO.output(p1RODA2B, GPIO.HIGH)
    GPIO.output(p2RODA2A, GPIO.LOW)
    GPIO.output(p2RODA2B, GPIO.HIGH)

def setbelokkiri(): #note: menambah nilai heading
    #Maju SET 1
    GPIO.output(p1RODA1A, GPIO.LOW)
    GPIO.output(p1RODA1B, GPIO.HIGH)
    GPIO.output(p2RODA1A, GPIO.LOW)
    GPIO.output(p2RODA1B, GPIO.HIGH)
    #Maju SET 2
    GPIO.output(p1RODA2A, GPIO.HIGH)
    GPIO.output(p1RODA2B, GPIO.LOW)
    GPIO.output(p2RODA2A, GPIO.HIGH)
    GPIO.output(p2RODA2B, GPIO.LOW)

def setbelokkanan(): #note: mengurangi nilai heading
    #Maju SET 1
    GPIO.output(p1RODA1A, GPIO.HIGH)
    GPIO.output(p1RODA1B, GPIO.LOW)
    GPIO.output(p2RODA1A, GPIO.HIGH)
    GPIO.output(p2RODA1B, GPIO.LOW)
    #Maju SET 2
    GPIO.output(p1RODA2A, GPIO.LOW)
    GPIO.output(p1RODA2B, GPIO.HIGH)
    GPIO.output(p2RODA2A, GPIO.LOW)
    GPIO.output(p2RODA2B, GPIO.HIGH)
#======================FUNCTION: SET MAJU / MUNDUR=======================<<

#======================FUNCTION: Movement Package=======================>>
def fiveTicks():
    if (now > starttime + runtime - 4) and (statetrigger1 == True):
        mx1 = mpu.mapxAHRS[-1]
        my1 = mpu.mapyAHRS[-1]
        statetrigger1 = False
    if (now > starttime + runtime - 3) and (statetrigger2 == True):
        mx2 = mpu.mapxAHRS[-1]
        my2 = mpu.mapyAHRS[-1]
        statetrigger2 = False
    if (now > starttime + runtime - 2) and (statetrigger3 == True):
        mx3 = mpu.mapxAHRS[-1]
        my3 = mpu.mapyAHRS[-1]
        mpu.vel = 0
        mpu.velAHRS = 0
        #setmaju()
        setmundur()
        statetrigger3 = False
    if (now > starttime + runtime - 1) and (statetrigger4 == True):
        mx4 = mpu.mapxAHRS[-1]
        my4 = mpu.mapyAHRS[-1]
        statetrigger4 = False
    if (now > starttime + runtime + 0) and (statetriggerf == True):
        #Stop Machine
        #shutdownSonar()
        mxf = mpu.mapxAHRS[-1]
        myf = mpu.mapyAHRS[-1]
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        pwm4.stop()
        statetriggerf = False
#======================FUNCTION: Movement Package=======================<<

#======================FUNCTION: Waypoint Creator=======================>>
# def jalanLurus():
#     dr = 1
#     golx.append( dr * np.cos(mpu.yaw*np.pi/180) )
#     goly.append( dr * np.sin(mpu.yaw*np.pi/180) )
#     kendali.setGoal(golx,goly)

# def jalanPersegi1():
#     gxt = 1 * np.cos(mpu.yaw*np.pi/180)
#     gyt = 1 * np.sin(mpu.yaw*np.pi/180)
#     golx.append(gxt)
#     goly.append(gyt)

#     gxt = gxt + 1 * np.cos((mpu.yaw*np.pi/180)-(0.5*np.pi))
#     golx.append(gxt)
#     goly.append(gyt)
#     golx.append(gxt)
#     goly.append(0)
#     golx.append(0)
#     goly.append(0)

# def jalanPersegi2():
#     dr = 1
#     golx.append(0)
#     goly.append(dr)
#     golx.append(dr)
#     goly.append(dr)
#     golx.append(dr)
#     goly.append(0)
#     golx.append(0)
#     goly.append(0)
#     kendali.setGoal(golx,goly)

#======================FUNCTION: Waypoint Creator=======================<<



#======================CLASS: PENGUKURAN JARAK OBSTACLE======================>>
class Sonar(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.range = 0
        self.start()

    #Fungsi Pengukuran Jarak
    def run(self):
        while True:
            GPIO.output(TRIG, True)
            time.sleep(0.2)
            GPIO.output(TRIG, False)
            while GPIO.input(ECHO) == False:
                rStart = time.time()
            while ( GPIO.input(ECHO) == True ):
                rEnd = time.time()
            rSig_time = rEnd-rStart
            #ori = 0.000058 mod = 0.000087
            rDist = rSig_time / 0.000058
            if rDist < 20:
                rDist = 0
            elif rDist > 200:
                rDist = 200
            #print('Obstacle distance: {} cm'.format(rDist))
            time.sleep(0.15)
            self.range = rDist

#======================CLASS: PENGUKURAN JARAK OBSTACLE======================<<

#======================FUNCTION: SETUP PARAMETER DAN FUNGSI ROBOT======================>>[3]
#Set STOP radius
STP = 50
#Set SPD multiplier, max = 5
mspd = 2
sepid = 10

#Set spd awal
pwmspd1 = sepid
pwmspd2 = sepid
pwmspd3 = sepid
pwmspd4 = sepid
#pwmspd = rangeMR() * mspd // 10
print('SPD CALCULATED {}, READY TO START'.format(pwmspd1))

def enginestart():
    pwm1.start(pwmspd1)
    pwm2.start(pwmspd2)
    pwm3.start(pwmspd3)
    pwm4.start(pwmspd4)
#======================FUNCTION: SETUP PARAMETER DAM FUNGSI ROBOT======================<<

#=======================CLASS: KENDALI===============================>>[4]
class Kendali:
    def __init__(self):
        self.errorPhi = 0
        self.p = 0
        self.w = 0
        self.rangeToObs = 0
        self.phiNew = 0
        self.phiOld = 0
        self.x = 0
        self.y = 0
        self.phi = 0
        self.ae = 0.2
        self.goalX = []
        self.goalY = []
        self.checkPointX = []
        self.checkPointY = []
        self.waypointX = 0
        self.waypointY = 0
        self.waypointN = 0
        self.waypointC = 0
        self.finish = False
        self.deltaX = 9999
        self.deltaY = 9999
        self.goalPhi = 0
        self.errorP = 0
        self.kP = 0.4
        self.pid = []
        self.v = 1
        self.maxSpeed = 0.42 #0.42m/s
        self.pwmMin = 0
        self.pwmMax = 100
        self.l = 24.5
        self.r = 3.5
        self.pitch = False

        self.vKanan = 0
        self.vKiri = 0
        self.pwmKiri = 0
        self.pwmKanan = 0

        self.nowAvoid = 0
        self.clearAvoid = True
        self.avoidTime = 1
        self.postAvoidTime = 3

        self.cd = 0

        self.pivot = True

        self.enginestart()

    def setGoal(self,x,y):
        self.goalX = x
        self.goalY = y
        self.waypointN = len(self.goalX)
        self.setWaypoint()

    def setWaypoint(self):
        self.pivot = True
        if self.waypointC > 0:
            self.setCheckPoint()
        if self.waypointC < self.waypointN:
            self.waypointX = self.goalX[self.waypointC]
            self.waypointY = self.goalY[self.waypointC]
            self.waypointC += 1
        else:
            self.enginekill()
            self.finish = True

    def setCheckPoint(self):
        self.checkPointX.append(self.x)
        self.checkPointY.append(self.y)

    def enginekill(self):
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        pwm4.stop()

    def kontrol(self,x,y,phi,obsRange,pitch):
        self.x = x
        self.y = y
        self.phi = phi
        self.rangeToObs = obsRange
        self.pitch = pitch

        if self.finish:
            self.enginekill()
            print('Finish')
        if (abs(self.deltaX) < self.ae) and (abs(self.deltaY) < self.ae):
            self.setWaypoint()
        # elif (self.rangeToObs < 100):
        #     self.exeObsAvoid()
        #     self.pivot = True
        #     print('obs')
        # elif (not self.clearAvoid):
        #     self.exePostAvoid()
        #     print('postobs')
        if (self.clearAvoid) and (not self.finish):
            self.exeGoToGoal()
            self.enginestart()
            print('gtg')
            print(f'Kanan: {self.pwmKanan}  Kiri:{self.pwmKiri}')

    def enginestart(self):
        pwm1.start(self.pwmKiri)
        pwm2.start(self.pwmKanan)
        pwm3.start(self.pwmKiri)
        pwm4.start(self.pwmKanan)

    def engineset(self):
        pwm1.ChangeDutyCycle(self.pwmKiri)
        pwm2.ChangeDutyCycle(self.pwmKanan)
        pwm3.ChangeDutyCycle(self.pwmKiri)
        pwm4.ChangeDutyCycle(self.pwmKanan)

    def setmaju(self):
        #Maju SET 1 roda kiri
        GPIO.output(p1RODA1A, GPIO.HIGH)
        GPIO.output(p1RODA1B, GPIO.LOW)
        GPIO.output(p2RODA1A, GPIO.HIGH)
        GPIO.output(p2RODA1B, GPIO.LOW)
        #Maju SET 2 roda kanan
        GPIO.output(p1RODA2A, GPIO.HIGH)
        GPIO.output(p1RODA2B, GPIO.LOW)
        GPIO.output(p2RODA2A, GPIO.HIGH)
        GPIO.output(p2RODA2B, GPIO.LOW)

    def setmundur(self):
        #Maju SET 1
        GPIO.output(p1RODA1A, GPIO.LOW)
        GPIO.output(p1RODA1B, GPIO.HIGH)
        GPIO.output(p2RODA1A, GPIO.LOW)
        GPIO.output(p2RODA1B, GPIO.HIGH)
        #Maju SET 2
        GPIO.output(p1RODA2A, GPIO.LOW)
        GPIO.output(p1RODA2B, GPIO.HIGH)
        GPIO.output(p2RODA2A, GPIO.LOW)
        GPIO.output(p2RODA2B, GPIO.HIGH)

    def setbelokkiri(self): #note: menambah nilai heading
        #Maju SET 1
        GPIO.output(p1RODA1A, GPIO.LOW)
        GPIO.output(p1RODA1B, GPIO.HIGH)
        GPIO.output(p2RODA1A, GPIO.LOW)
        GPIO.output(p2RODA1B, GPIO.HIGH)
        #Maju SET 2
        GPIO.output(p1RODA2A, GPIO.HIGH)
        GPIO.output(p1RODA2B, GPIO.LOW)
        GPIO.output(p2RODA2A, GPIO.HIGH)
        GPIO.output(p2RODA2B, GPIO.LOW)

    def setbelokkanan(self): #note: mengurangi nilai heading
        #Maju SET 1
        GPIO.output(p1RODA1A, GPIO.HIGH)
        GPIO.output(p1RODA1B, GPIO.LOW)
        GPIO.output(p2RODA1A, GPIO.HIGH)
        GPIO.output(p2RODA1B, GPIO.LOW)
        #Maju SET 2
        GPIO.output(p1RODA2A, GPIO.LOW)
        GPIO.output(p1RODA2B, GPIO.HIGH)
        GPIO.output(p2RODA2A, GPIO.LOW)
        GPIO.output(p2RODA2B, GPIO.HIGH)

    def errorCalculate(self):
        self.phiOld = self.phiNew

        self.phiNew = np.arctan2(np.sin(self.phi),np.cos(self.phi))
        self.deltaX = self.waypointX - self.x
        self.deltaY = self.waypointY - self.y
        self.goalPhi = np.arctan2(self.deltaY, self.deltaX)
        self.errorPhi = self.goalPhi - self.phiNew
        self.errorPhi = np.arctan2(np.sin(self.errorPhi),np.cos(self.errorPhi))

        self.errorP = self.errorPhi
        self.p = self.kP * self.errorP
        # if self.pitch:
        #     self.cd = 8
        # if self.cd > 0:
        #     self.cd -= 1
        #self.p = 0
        self.pid.append(self.p)
        self.w = self.p

    def exeGoToGoal(self):
        self.goToGoal()
        self.controlV()
        self.executeMotor()

    def goToGoal(self):
        self.errorCalculate()
        self.uniToDiff()

    def uniToDiff(self):
        self.vKanan = (2 * self.v + self.w * self.l) / (2 * self.r)
        self.vKiri = (2 * self.v - self.w * self.l) / (2 * self.r)

    def controlV(self):
        velRLMax = max(self.vKanan, self.vKiri)
        velRLMin = min(self.vKanan, self.vKiri)
        if (velRLMax > self.maxSpeed):
            self.vKanan = self.vKanan - (velRLMax - self.maxSpeed)
            self.vKiri = self.vKiri - (velRLMax - self.maxSpeed)
        elif (velRLMin < (-self.maxSpeed)):
            self.vKanan = self.vKanan - (velRLMin + self.maxSpeed)
            self.vKiri = self.vKiri - (velRLMin + self.maxSpeed)

    def executeMotor(self):
        if ((self.vKanan > 0) and (self.vKiri > 0)):
            self.setupMaju()
        elif ((self.vKanan < 0) and (self.vKiri < 0)):
            self.vKanan = -self.vKanan
            self.vKiri = -self.vKiri
            self.setupMundur()
        elif ((self.vKanan < 0) and (self.vKiri > 0)):
            self.vKanan = -self.vKanan
            self.setupKanan()
        elif ((self.vKanan > 0) and (self.vKiri < 0)):
            self.vKiri = -self.vKiri
            self.setupKiri()

    def setupMaju(self):
        self.pwmKanan = np.interp(self.vKanan,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pwmKiri = np.interp(self.vKiri,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pivot = False
        self.setmaju()
        self.engineset()

    def setupMundur(self):
        self.pwmKanan = np.interp(self.vKanan,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pwmKiri = np.interp(self.vKiri,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pivot = False
        self.setmundur()
        self.engineset()

    def setupKanan(self):
        self.pwmKanan = np.interp(self.vKanan,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pwmKiri = np.interp(self.vKiri,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        # self.pivot = True
        self.setbelokkanan()
        self.engineset()

    def setupKiri(self):
        self.pwmKanan = np.interp(self.vKanan,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        self.pwmKiri = np.interp(self.vKiri,[0,self.maxSpeed],[self.pwmMin,self.pwmMax])
        # self.pivot = True
        self.setbelokkiri()
        self.engineset()

    def exeObsAvoid(self):
        self.nowAvoid = time.perf_counter()
        self.clearAvoid = False
        self.pwmKanan = self.pwmMax
        self.pwmKiri = self.pwmMax
        self.errorCalculate()
        self.setbelokkanan()
        self.engineset()

    def exePostAvoid(self):
        checkNow = time.perf_counter()
        self.errorCalculate()
        if (checkNow > self.nowAvoid + self.avoidTime + self.postAvoidTime):
            self.clearAvoid = True
            self.pivot = True
        elif (checkNow > self.nowAvoid + self.avoidTime):
            self.pwmKanan = self.pwmMax
            self.pwmKiri = self.pwmMax
            self.pivot = False
            self.setmaju()
            self.engineset()
        else:
            self.pwmKanan = self.pwmMax
            self.pwmKiri = self.pwmMax
            self.setbelokkanan()
            self.engineset()

#=======================CLASS: KENDALI===============================<<[4]


#=======================CLASS: MPU===============================>>[4]
class MPU:
    def __init__(self, gyro, acc, mag, tau): #1 deklarasi variable class
        # Class / object / constructor setup
        self.ax = None; self.ay = None; self.az = None;
        self.gx = None; self.gy = None; self.gz = None;
        self.mx = None; self.my = None; self.mz = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.magXcal = 0; self.magXbias = 0; self.magXscale = 0;
        self.magYcal = 0; self.magYbias = 0; self.magYscale = 0;
        self.magZcal = 0; self.magZbias = 0; self.magZscale = 0;

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.xt1 = 0
        self.yt1 = 0
        self.xt1AHRS = 0
        self.yt1AHRS = 0
        self.vel = 0
        self.velx = 0
        self.velAHRS = 0
        self.velAHRSOld = 0
        self.dist = 0
        self.distKalman = 0
        self.distAHRS = 0
        self.distFullAHRS = 0
        self.dispAHRS = 0
        self.distAHRSMul = 4
        self.robotHeading = 0


        self.posisi = []
        self.mapx = []
        self.mapy = []
        self.posisiAHRS = []
        self.mapxAHRS = []
        self.mapyAHRS = []
        self.mapxAHRS = []
        self.mapyAHRS = []
        self.trackingx = []
        self.trackingy = []
        self.trackingz = []
        self.trackingrawx = []
        self.trackingrawy = []
        self.trackingrawz = []
        self.trackingahrsx = []
        self.trackingahrsy = []
        self.trackingahrsz = []
        self.arrayaccx = []
        self.arrayaccy = []
        self.arrayaccz = []
        self.arraydeltat = []
        self.pitched = []
        self.arrayVelAHRS = []

        self.ptch = False

        self.grafreex= 0
        self.grafreey= 0
        self.grafreez= 0

        self.grafreexOld = 0
        self.grafreeyOld = 0
        self.grafreezOld = 0

        self.nax = 0
        self.nay = 0
        self.naz = 0
        self.alpha = 0.1

        self.G = 9.8066

        self.velMaxDrift = -9999
        self.velMinDrift = 9999

        self.dtVelDist = 0
        self.dtTimeN = 0
        self.dtTimer = 0
        self.tau = tau

        self.q = [1,0,0,0]
        self.beta = 1

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor,  self.accHex  = self.accelerometerSensitivity(acc)
        self.magScaleFactor,  self.magHex  = self.magnetometerSensitivity(mag)

        self.bus = smbus.SMBus(1)

        self.MPU9250_ADDRESS  = 0x68
        self.AK8963_ADDRESS   = 0x0C
        self.WHO_AM_I_MPU9250 = 0x75
        self.WHO_AM_I_AK8963  = 0x00

        self.AK8963_XOUT_L    = 0x03
        self.AK8963_CNTL      = 0x0A
        self.AK8963_CNTL2     = 0x0B
        self.USER_CTRL        = 0x6A
        self.I2C_SLV0_DO      = 0x63

        self.AK8963_ASAX      = 0x10
        self.I2C_MST_CTRL     = 0x24
        self.I2C_SLV0_ADDR    = 0x25
        self.I2C_SLV0_REG     = 0x26
        self.I2C_SLV0_CTRL    = 0x27
        self.EXT_SENS_DATA_00 = 0x49
        self.GYRO_CONFIG      = 0x1B
        self.ACCEL_XOUT_H     = 0x3B
        self.PWR_MGMT_1       = 0x6B
        self.ACCEL_CONFIG     = 0x1C

    def gyroSensitivity(self, x): #2 set tingkat sensitifitas gyroscope dalam deg/s
        # Create dictionary with standard value of 500 deg/s
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x): #3 set tingkat sensitifitas accelerometer dlm g. output dalam m/s2
        # Create dictionary with standard value of 4 g
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x,[8192.0,  0x08])

    def magnetometerSensitivity(self, x): #4 set tingkat sensitifitas magnetometer
        # Create dictionary with standard value of 16 bit
        return {
            14:  [10.0*4912.0/8190.0,  0x06],
            16:  [10.0*4912.0/32760.0, 0x16],
        }.get(x,[10.0*4912.0/32760.0,  0x16])

    def setUpIMU(self): #5 fungsi koneksi dengan accelerometer
        # Check to see if there is a good connection with the MPU 9250
        try:
            whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.WHO_AM_I_MPU9250)
        except:
            print('whoAmI IMU read failed')
            return

        if (whoAmI == 0x71):
            # Connection is good! Activate/reset the IMU
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.PWR_MGMT_1, 0x00)

            # Configure the accelerometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.ACCEL_CONFIG, self.accHex)

            # Configure the gyro
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.GYRO_CONFIG, self.gyroHex)

            # Display message to user
            print("MPU set up:")
            print('\tAccelerometer: ' + str(hex(self.accHex)) + ' ' + str(self.accScaleFactor))
            print('\tGyroscope: ' + str(hex(self.gyroHex)) + ' ' + str(self.gyroScaleFactor) + "\n")
        else:
            # Bad connection or something went wrong
            print("IMU WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x71))

    def setUpMAG(self):#6 fungsi koneksi dengan accelerometer
        # Initialize connection with mag for a WHO_AM_I test
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.USER_CTRL, 0x20);                              # Enable I2C Master mode
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_MST_CTRL, 0x0D);                           # I2C configuration multi-master I2C 400KHz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.WHO_AM_I_AK8963);           # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                          # Enable I2C and transfer 1 byte
        time.sleep(0.05)

        # Check to see if there is a good connection with the mag
        try:
            whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00)
        except:
            print('whoAmI MAG read failed')
            return

        if (whoAmI == 0x48):
            # Connection is good! Begin the true initialization
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL2);        # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x01);                      # Reset AK8963
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x00);                      # Power down magnetometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x0F);                      # Enter fuze mode
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);   # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_ASAX);              # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x83);                         # Enable I2C and read 3 bytes
            time.sleep(0.05)

            # Read the x, y, and z axis calibration values
            try:
                rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00, 3)
            except:
                print('Reading MAG x y z calibration values failed')
                return

            # Convert values to something more usable
            self.magXcal =  float(rawData[0] - 128)/256.0 + 1.0;
            self.magYcal =  float(rawData[1] - 128)/256.0 + 1.0;
            self.magZcal =  float(rawData[2] - 128)/256.0 + 1.0;

            # Flush the sysem
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x00);                      # Power down magnetometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte

            # Configure the settings for the mag
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, self.magHex);               # Set magnetometer for 14 or 16 bit continous 100 Hz sample rates
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);               # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                          # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            # Display results to user
            print("MAG set up:")
            print("\tMagnetometer: " + hex(self.magHex) + " " + str(round(self.magScaleFactor,3)) + "\n")
        else:
            # Bad connection or something went wrong
            print("MAG WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x48))

    def eightBit2sixteenBit(self, l, h): #7 fungsi koneksi dengan accelerometer
        # Shift the low and high byte into a 16 bit number
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def readRawIMU(self): #8 mengambil nilai raw accelerometer/gyro kedalam ax gx
        # Read 14 raw values [High Low] as temperature falls between the accelerometer and gyro registries
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.ACCEL_XOUT_H, 14)
        except:
            print('Read raw IMU data failed')

        # Convert the raw values to something a little more useful (middle value is temperature)
        self.ax = self.eightBit2sixteenBit(rawData[1], rawData[0])
        self.ay = self.eightBit2sixteenBit(rawData[3], rawData[2])
        self.az = self.eightBit2sixteenBit(rawData[5], rawData[4])

        self.gx = self.eightBit2sixteenBit(rawData[9], rawData[8])
        self.gy = self.eightBit2sixteenBit(rawData[11], rawData[10])
        self.gz = self.eightBit2sixteenBit(rawData[13], rawData[12])

    def readRawMag(self): #9 mengambil nilai raw magnetometer kedalam mx
        # Prepare to request values
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_XOUT_L);             # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x87);                          # Enable I2C and read 7 bytes
        time.sleep(0.02)

        # Read 7 values [Low High] and one more byte (overflow check)
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00, 7)
        except:
            print('Read raw MAG data failed')

        # If overflow check passes convert the raw values to something a little more useful
        if not (rawData[6] & 0x08):
            self.mx = self.eightBit2sixteenBit(rawData[0], rawData[1])
            self.my = self.eightBit2sixteenBit(rawData[2], rawData[3])
            self.mz = self.eightBit2sixteenBit(rawData[4], rawData[5])

    def calibrateGyro(self, N): #10 fungsi untuk kalibrasi gyroscope. nilai awal dijadikan ofset
        # Display message
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        # Take N readings for each coordinate and add to itself
        for ii in range(N):
            self.readRawIMU()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        # Find average offset value
        self.gyroXcal /= N
        self.gyroYcal /= N
        self.gyroZcal /= N

        # Display message and restart timer for comp filter
        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal,1)))
        print("\tY axis offset: " + str(round(self.gyroYcal,1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")

        # Start the timer
        self.dtTimer = time.time()

    def calibrateMag(self, N): #11 fungsi untuk kalibrasi magnetometer. dilakukan dengan koreksi hard dan soft iron (normalisasi)
        # Local calibration variables
        magBias = [0, 0, 0]
        magScale = [0, 0, 0]
        magMin = [32767, 32767, 32767]
        magMax = [-32767, -32767, -32767]
        magTemp = [0, 0, 0]

        # Take N readings of mag data
        for ii in range(N):
            # Read fresh values and assign to magTemp
            self.readRawMag()
            magTemp = [self.mx, self.my, self.mz]

            # Adjust the max and min points based off of current reading
            for jj in range(3):
                if (magTemp[jj] > magMax[jj]):
                    magMax[jj] = magTemp[jj]
                if (magTemp[jj] < magMin[jj]):
                    magMin[jj] = magTemp[jj]

            # Display some info to the user
            print(str(self.mx)+','+str(self.my)+','+str(self.mz))

            # Small delay before next loop (data available every 10 ms or 100 Hz)
            time.sleep(0.012)

        # Get hard iron correction
        self.magXbias = ((magMax[0] + magMin[0])/2) * self.magScaleFactor * self.magXcal
        self.magYbias = ((magMax[1] + magMin[1])/2) * self.magScaleFactor * self.magYcal
        self.magZbias = ((magMax[2] + magMin[2])/2) * self.magScaleFactor * self.magZcal

        # Get soft iron correction estimate
        magXchord = (magMax[0] - magMin[0])/2
        magYchord = (magMax[1] - magMin[1])/2
        magZchord = (magMax[2] - magMin[2])/2

        avgChord = (magXchord + magYchord + magZchord)/3

        self.magXscale = avgChord/magXchord
        self.magYscale = avgChord/magYchord
        self.magZscale = avgChord/magZchord

    def calibrateMagGuide(self):#12 fungsi untuk kalibrasi magnetometer. dilakukan dengan koreksi hard dan soft iron (normalisasi)
        # Display message
        print("Magnetometer calibration. Wave and rotate device in a figure eight until notified.\n")
        time.sleep(3)

        # Run the first calibration
        self.calibrateMag(3000)

        # Display results to user
        print("\nCalibration complete:")
        print("\tmagXbias = " + str(round(self.magXbias,3)))
        print("\tmagYbias = " + str(round(self.magYbias,3)))
        print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

        print("\tmagXscale = " + str(round(self.magXscale,3)))
        print("\tmagYscale = " + str(round(self.magYscale,3)))
        print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

        # Give more instructions to the user
        print("Place above values in magCalVisualizer.py and magCalSlider.py")
        print("Recording additional 1000 data points to verify the calibration")
        print("Repeat random figure eight pattern and rotations...\n")
        time.sleep(3)

        # Run the scecond calibration
        self.calibrateMag(1000)

        # Provide final instructions
        print("\nCopy the raw values into data.txt")
        print("Run magCalVisualizer.py and magCalSlider.py to validate calibration success")
        print("See the README for more information")
        print("Also compare the second calibration to the first:")

        # Display the second results
        print("\tmagXbias = " + str(round(self.magXbias,3)))
        print("\tmagYbias = " + str(round(self.magYbias,3)))
        print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

        print("\tmagXscale = " + str(round(self.magXscale,3)))
        print("\tmagYscale = " + str(round(self.magYscale,3)))
        print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

        # End program
        print("Terminating program now!")
        quit()

    def setMagCalibration(self, bias, scale):#12 memasukan nilai hasil kalibrasi magnetometer.
        # Set the bias variables in all 3 axis
        self.magXbias = bias[0]
        self.magYbias = bias[1]
        self.magZbias = bias[2]

        # Set the scale variables in all 3 axis
        self.magXscale = scale[0]
        self.magYscale = scale[1]
        self.magZscale = scale[2]

    def processValues(self):#13 memunculkan nilai gyro gx (degree/second), accelerometer ax(g), magnetometer mx(miligauss)
        # Update the raw data
        self.readRawIMU()
        self.readRawMag()

        # Subtract the offset calibration values for the gyro
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert the gyro values to degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert the accelerometer values to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

        # Process mag values in milliGauss
        # Include factory calibration per data sheet and user environmental corrections
        self.mx = self.mx * self.magScaleFactor * self.magXcal - self.magXbias
        self.my = self.my * self.magScaleFactor * self.magYcal - self.magYbias
        self.mz = self.mz * self.magScaleFactor * self.magZcal - self.magZbias

        self.mx *= self.magXscale
        self.my *= self.magYscale
        self.mz *= self.magZscale

    def compFilter(self):#13 memunculkan nilai Roll Pitch Yaw menggunakan CompFilter.
        # Get the processed values from IMU
        self.processValues()

        # Get delta time and record time for next call
        dt = time.time() - self.dtTimer
        self.dtTimer = time.time()

        # Acceleration vector angle
        accRoll = math.degrees(math.atan2(self.ay, self.az))
        accPitch = math.degrees(math.atan2(self.ax, self.az))

        # Gyro integration angle
        self.gyroRoll += self.gx * dt
        self.gyroPitch -= self.gy * dt
        self.gyroYaw += self.gz * dt

        # Comp filter
        self.roll = (self.tau)*(self.roll + self.gx*dt) + (1-self.tau)*(accRoll)
        self.pitch = (self.tau)*(self.pitch - self.gy*dt) + (1-self.tau)*(accPitch)

        # Convert roll and pitch to radians for heading calculation
        rollRads = math.radians(self.roll)
        pitchRads = math.radians(self.pitch)

        # Reassign mag X and Y values in accordance to MPU-9250
        Mx = self.my
        My = self.mx
        Mz = self.mz

        # Normalize the values
        norm = math.sqrt(Mx * Mx + My * My + Mz * Mz)
        Mx1 = Mx / norm
        My1 = My / norm
        Mz1 = Mz / norm

        # Apply tilt compensation
        Mx2 = Mx1*math.cos(pitchRads) + Mz1*math.sin(pitchRads)
        My2 = Mx1*math.sin(rollRads)*math.sin(pitchRads) + My1*math.cos(rollRads) - Mz1*math.sin(rollRads)*math.cos(pitchRads)
        Mz2 = -Mx1*math.cos(rollRads)*math.sin(pitchRads) + My1*math.sin(rollRads) + Mz1*math.cos(rollRads)*math.cos(pitchRads)

        # Heading calculation
        if ((Mx2 > 0) and (My2 >=0)):
            self.yaw = math.degrees(math.atan(My2/Mx2))
        elif (Mx2 < 0):
            self.yaw = 180 + math.degrees(math.atan(My2/Mx2))
        elif ((Mx2 > 0) and (My2 <= 0)):
            self.yaw = 360 + math.degrees(math.atan(My2/Mx2))
        elif ((Mx2 == 0) and (My2 < 0)):
            self.yaw = 90
        elif ((Mx2 == 0) and (My2 > 0)):
            self.yaw = 270
        else:
            print('Error')

        # Print results to screen
        print('R: {:<8.1f} P: {:<8.1f} Y: {:<8.1f}'.format(self.roll,self.pitch,self.yaw))
        # print(math.sqrt(Mx2 * Mx2 + My2 * My2 + Mz2 * Mz2))
        # print(math.sqrt(Mx1 * Mx1 + My1 * My1 + Mz1 * Mz1))

    def madgwickFilter(self, ax, ay, az, gx, gy, gz, mx, my, mz, deltat): #14 mencari nilai qurternion menggunakan madgwick filter
      # Quaternion values
      q1 = self.q[0]
      q2 = self.q[1]
      q3 = self.q[2]
      q4 = self.q[3]

      # Auxiliary variables
      q1x2 = 2 * q1
      q2x2 = 2 * q2
      q3x2 = 2 * q3
      q4x2 = 2 * q4
      q1q3x2 = 2 * q1 * q3
      q3q4x2 = 2 * q3 * q4
      q1q1 = q1 * q1
      q1q2 = q1 * q2
      q1q3 = q1 * q3
      q1q4 = q1 * q4
      q2q2 = q2 * q2
      q2q3 = q2 * q3
      q2q4 = q2 * q4
      q3q3 = q3 * q3
      q3q4 = q3 * q4
      q4q4 = q4 * q4

      # Normalize accelerometer measurement
      norm = math.sqrt(ax * ax + ay * ay + az * az)
      if norm is 0: return
      ax /= norm
      ay /= norm
      az /= norm

      #temporary put norm ax into nax
      self.nax = ax
      self.nay = ay
      self.naz = az

      # Normalize magnetometer measurement
      norm = math.sqrt(mx * mx + my * my + mz * mz)
      if norm is 0: return
      mx /= norm
      my /= norm
      mz /= norm

      # Reference direction of Earth's magnetic field
      hx = mx * q1q1 - (2*q1*my) * q4 + (2*q1*mz) * q3 + mx * q2q2 + q2x2 * my * q3 + q2x2 * mz * q4 - mx * q3q3 - mx * q4q4
      hy = (2*q1*mx) * q4 + my * q1q1 - (2*q1*mz) * q2 + (2*q2*mx) * q3 - my * q2q2 + my * q3q3 + q3x2 * mz * q4 - my * q4q4
      bx_2 = math.sqrt(hx * hx + hy * hy)
      bz_2 = -(2*q1*mx) * q3 + (2*q1*my) * q2 + mz * q1q1 + (2*q2*mx) * q4 - mz * q2q2 + q3x2 * my * q4 - mz * q3q3 + mz * q4q4
      bx_4 = 2 * bx_2
      bz_4 = 2 * bz_2

      # Gradient decent algorithm corrective step
      s1 = -q3x2 * (2 * q2q4 - q1q3x2 - ax) + q2x2 * (2 * q1q2 + q3q4x2 - ay) - bz_2 * q3 * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (-bx_2 * q4 + bz_2 * q2) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + bx_2 * q3 * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
      s2 = q4x2 * (2 * q2q4 - q1q3x2 - ax) + q1x2 * (2 * q1q2 + q3q4x2 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + bz_2 * q4 * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (bx_2 * q3 + bz_2 * q1) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + (bx_2 * q4 - bz_4 * q2) * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
      s3 = -q1x2 * (2 * q2q4 - q1q3x2 - ax) + q4x2 * (2 * q1q2 + q3q4x2 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-bx_4 * q3 - bz_2 * q1) * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (bx_2 * q2 + bz_2 * q4) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + (bx_2 * q1 - bz_4 * q3) * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
      s4 = q2x2 * (2 * q2q4 - q1q3x2 - ax) + q3x2 * (2 * q1q2 + q3q4x2 - ay) + (-bx_4 * q4 + bz_2 * q2) * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (-bx_2 * q1 + bz_2 * q3) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + bx_2 * q2 * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)

      # Normalize step magnitude
      norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
      s1 /= norm
      s2 /= norm
      s3 /= norm
      s4 /= norm

      # Compute rate of change of quaternion
      qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
      qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
      qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
      qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

      # Integrate to yield quaternion
      q1 += qDot1 * deltat
      q2 += qDot2 * deltat
      q3 += qDot3 * deltat
      q4 += qDot4 * deltat

      # Normalize quaternion
      norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
      self.q[0] = q1 / norm
      self.q[1] = q2 / norm
      self.q[2] = q3 / norm
      self.q[3] = q4 / norm

    def attitudeEuler(self):#15 memunculkan nilai Roll Pitch Yaw menggunakan quarternion.
        # Get the data from the matrix
        a12 = 2 * (self.q[1] * self.q[2] + self.q[0] * self.q[3])
        a22 = self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]
        a31 = 2 * (self.q[0] * self.q[1] + self.q[2] * self.q[3])
        a32 = 2 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])
        a33 = self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]

        # Perform the conversion to euler
        self.roll  = math.degrees(math.atan2(a31, a33))
        self.pitch = -math.degrees(math.asin(a32))
        self.yaw   = math.degrees(math.atan2(a12, a22))

        # Declination 14 deg 7 minutes at Edmonton May 31, 2019. Bound yaw between [0 360]
        self.yaw += 14.1
        if (self.yaw < 0): self.yaw += 360

        # Free Gravity Acc Old
        self.grafreexOld = self.grafreex
        self.grafreeyOld = self.grafreey
        self.grafreezOld = self.grafreez

        # Free Gravity acceleration
        self.grafreex = (self.nax - a32) * self.G
        self.grafreey = (self.nay - a31) * self.G
        self.grafreez = (self.naz - a33) * self.G

        # Low Pass Filter
        self.grafreey = self.grafreeyOld + (self.alpha * (self.grafreey - self.grafreeyOld))

        # Print data
        print('R: {:<8.1f} P: {:<8.1f} Y: {:<8.1f}'.format(self.roll,self.pitch,self.yaw))

    def speedCalculate(self, deltat): #Melakukan kalkulasi kecepatan dari acceleromter
        #if (self.grafreex < self.velMinDrift):
        tnay = self.nay * self.G
        tnax = self.nax * self.G
        #self.nay = self.nay * self.G
        #self.nax = self.nax * self.G
        self.vel += (tnay * deltat)
        self.velx += (tnax *deltat)
        if self.velAHRS < 0:
            self.grafreey = abs(self.grafreey)
        self.velAHRSOld = self.velAHRS
        # if self.ptch:
        #     self.velAHRS += (self.grafreey *deltat) * 0.01
        # else:
        self.velAHRS += (self.grafreey *deltat)
        self.arrayVelAHRS.append(self.velAHRS)
        #low pass filter
        #self.velAHRS = self.velAHRSOld + self.alpha * (self.velAHRS - self.velAHRSOld)


        #self.trackingx.append(self.grafreex)
        #self.trackingz.append(self.ax * self.G)
        self.arraydeltat.append(deltat)
        self.arrayaccy.append(tnay)
        self.arrayaccx.append(tnax)
        self.trackingy.append(self.vel)
        self.trackingx.append(self.velx)
        self.trackingz.append(self.velAHRS)
        self.trackingahrsy.append(self.grafreey)
        #print('Vel: {:.4f} Acc:{:.4f}'.format(self.vel, self.grafreex))

    def distanceCalculate(self, deltat): #Melakukan kalkulasi distance dari acceleromter
        #s = s0 + v0 * t + 1/2 a t^2
        self.dist = (self.vel * deltat) + (0.5 * self.vel * deltat*deltat)
        self.distKalman += (self.vel * deltat) + (0.5 * self.vel * deltat*deltat)
        self.distAHRS = ((self.velAHRSOld * deltat) + (0.5 * self.grafreey * deltat*deltat)) * self.distAHRSMul
        self.dispAHRS += self.distAHRS
        self.distFullAHRS += abs(self.distAHRS)
        #self.dist = 1
        #print('Dist: {}'.format(self.vel))

    def printSpeedDistance(self): #Print speed dan distance ke monitor
        print('Vel: {:.8f} Dist: {:.2f}'.format(self.vel, self.dist))
        #print('Dist: {}'.format(self.vel))

    def positionTracking(self):# Melakukan tracking/mapping terhadap gerakan robot.
        #print("executed")
        #self.pitched.append = self.pitched
        #self.hold =
        self.heading = self.yaw * np.pi / 180 #heading in rad. yaw in deg
        self.robotHeading = self.heading
        hx = self.xt1 + self.dist * np.cos(self.heading)
        hy = self.yt1 + self.dist * np.sin(self.heading)
        self.mapx.append(hx)
        self.mapy.append(hy)
        self.posisi.append([hx,hy])
        self.xt1 = hx
        self.yt1 = hy
        #self.trackingx.append(self.ax)

    def positionTrackingAHRS(self):# Melakukan tracking/mapping terhadap gerakan robot.
        #print("executed")
        self.pitched.append(self.pitch)
        self.hold = self.heading
        self.heading = self.yaw * np.pi / 180 #heading in rad. yaw in deg
        if self.pitch < 0:
            # self.heading = self.hold
            self.ptch = True
        else:
            self.ptch = False
        self.robotHeading = self.heading
        hx = self.xt1AHRS + self.distAHRS * np.cos(self.heading)
        hy = self.yt1AHRS + self.distAHRS * np.sin(self.heading)
        self.mapxAHRS.append(hx)
        self.mapyAHRS.append(hy)
        self.posisiAHRS.append([hx,hy])
        self.xt1AHRS = hx
        self.yt1AHRS = hy
        #self.trackingx.append(self.ax)

    '''def driftCalculator(self, deltat):# Menghitung noise acceleration
        #self.vel += (self.grafreex * deltat)
        if self.grafreex > self.velMaxDrift:
            self.velMaxDrift = self.grafreex
            print(self.velMaxDrift)
        elif self.grafreex < self.velMinDrift:
            self.velMinDrift = self.grafreex'''

#=====================CLASS: MPU=====================<<

#======================Main Program=======================>>[5]
def main():
    # Set up class
    gyro = 500      # 250, 500, 1000, 2000 [deg/s]
    acc = 4         # 2, 4, 7, 16 [g]
    mag = 16        # 14, 16 [bit]
    tau = 0.98
    mpu = MPU(gyro, acc, mag, tau)
    c = 0
    prepphase1 = 4
    prepphase2 = 8
    runtime = 10
    rangeR = 0
    golx = []
    goly = []
    tempe = 0
    tempe2 = 0
    dps = 0.42
    ce = 0

    #infinite Looping Sonar
    sonar = Sonar()

    mpu.mapx.append(mpu.xt1)
    mpu.mapy.append(mpu.yt1)

    # Set up the IMU and mag sensors
    mpu.setUpIMU()
    mpu.setUpMAG()

    # Set up Kendali
    #golx = 0
    #goly = -1
    kendali = Kendali()
    #kendali.setGoal(golx,goly)

    # Calibrate the mag or provide values that have been verified with the visualizer #edit bias disini
    #mpu.calibrateMagGuide()
    bias = [290.085, 989.65, 30.784]
    scale = [1.055, 0.937, 1.015]
    mpu.setMagCalibration(bias, scale)

    # Calibrate the gyro with N points
    mpu.calibrateGyro(1000)

    # Set timer
    lastUpdate = time.perf_counter()
    starttime = lastUpdate + 100
    preptime = lastUpdate
    starttrigger1 = True
    starttrigger2 = True

    statetrigger1 = True
    statetrigger2 = False
    statetrigger3 = False
    statetrigger4 = False
    statetrigger5 = False
    statetrigger6 = False
    statetriggerf = True


    # Run Robot
    #setmaju()
    #setbelokkiri()
    #setmundur()

        # Run until stopped
    try:
        while(True):
            # Get new values
            mpu.processValues()
            #use these 3 lines for mode2
            now2 = time.perf_counter()
            deltat2 = ((now2 - lastUpdate))
            lastUpdate2 = now2

            for ii in range(10):
                # Integration timer
                now = time.perf_counter()
                deltat = ((now - lastUpdate))
                lastUpdate = now

                # Run the sensor fusion (must be aligned to NED and righthand rule).
                # For my sensor IMU x axis is aligned with mag y axis
                # IMU y axis is aligned with mag x axis and both need to be inversed
                # IMU and mag z axis aligned however gyro z axis needs to be inversed
                #MPU mode1
                mpu.madgwickFilter(mpu.ax, mpu.ay, mpu.az, \
                    math.radians(mpu.gx), math.radians(mpu.gy), math.radians(mpu.gz), \
                    mpu.my, mpu.mx, mpu.mz, deltat)
                #MPU mode2(original)
                '''mpu.madgwickFilter(mpu.ax, -mpu.ay, mpu.az, \
                              math.radians(mpu.gx), -math.radians(mpu.gy), -math.radians(mpu.gz), \
                              mpu.my, -mpu.mx, mpu.mz, deltat)'''
                #mode1: kalkulasi diatas
                #if not starttrigger1:
                    #mpu.driftCalculator(deltat)
                    #mpu.distanceCalculate(deltat)
                    #mpu.positionTracking()
                #if not starttrigger2:
                    #mpu.speedCalculate(deltat)


            #if not starttrigger1:
                #mpu.driftCalculator(deltat2)
                #mpu.distanceCalculate(deltat2)
                #mpu.positionTracking()
                #mpu.positionTrackingAHRS()
            if not starttrigger2:
                mpu.speedCalculate(deltat2)
                mpu.distanceCalculate(deltat2)
                mpu.positionTracking()
                mpu.positionTrackingAHRS()
                rangeR = sonar.range
                print(f'Range: {rangeR:8.1f} Heading: {mpu.robotHeading:8.1f} Yaw: {mpu.yaw:8.1f}')

            # Print results to screen
            mpu.attitudeEuler()
            #mode2: kalkulasi dibawah

            #mpu.compFilter()

            # Mapping and speed
            if (now > preptime + prepphase1):
                if starttrigger1:
                    print("<<<<<<<<<<Tahap 1>>>>>>>>>>>")
                    starttrigger1 = False
            if (now > preptime + prepphase2):
                if starttrigger2:
                    print("<<<<<<<<<<Tahap 2>>>>>>>>>>>")
                    mpu.vel = 0
                    mpu.velAHRS = 0
                    #========setgoal==========
                    # dr = 3
                    # golx.append(0)
                    # goly.append(dr)
                    # golx.append(dr)
                    # goly.append(dr)
                    # golx.append(dr)
                    # goly.append(0)
                    # golx.append(0)
                    # goly.append(0)
                    # kendali.setGoal(golx,goly)
                    dr = 1
                    golx.append( dr * np.cos(mpu.yaw*np.pi/180) )
                    goly.append( dr * np.sin(mpu.yaw*np.pi/180) )
                    kendali.setGoal(golx,goly)
                    #=======setgoal===========

                    starttime = now
                    #enginestart()
                starttrigger2 = False
                #mpu.printSpeedDistance()
                #Move ganjil = kiri
                #pwm1.start(pwmspd1)
                #pwm2.start(pwmspd2)
                #pwm3.start(pwmspd3)
                #pwm4.start(pwmspd4)
            #mpu.positionTracking()

            #======================KENDALI===========================
            if (now > starttime):
                kendali.kontrol(mpu.mapxAHRS[-1], mpu.mapyAHRS[-1], mpu.robotHeading, sonar.range, mpu.ptch)
                if kendali.pivot:
                    mpu.velAHRS = 0
                    mpu.vel = 0
                ce += 1
                cet = now
                # if ce == 1000:
                #     break

            #======================MOTOR DRIVER======================

            # if (now > starttime + runtime - 3) and (statetrigger2 == True):
            #     mpu.velAHRS = 0
            #     if statetrigger1 == True:
            #         statetrigger1 = False
            #         markedHeading = mpu.yaw
            #         print(markedHeading)
            #         mx1 = mpu.mapxAHRS[-1]
            #         my1 = mpu.mapyAHRS[-1]
            #         if ((markedHeading + 90) > 359.9):
            #             statetrigger3 = False
            #         setbelokkiri()
            #     if statetrigger3:
            #         if (mpu.yaw >= (markedHeading + 90)):
            #             statetrigger2 = False
            #     else:
            #         if ((mpu.yaw) >= (markedHeading - 270)):
            #             statetrigger2 = False
            # if statetrigger2 == False:
            #     if statetrigger3 == True:
            #         mpu.velAHRS = 0
            #         statetrigger3 = False
            #         mx2 = mpu.mapxAHRS[-1]
            #         my2 = mpu.mapyAHRS[-1]
            #     setmaju()


            if (now > starttime + 1) and (statetrigger1 == True):
                mx1 = mpu.mapxAHRS[-1]
                my1 = mpu.mapyAHRS[-1]
                tempe = mpu.dispAHRS
                #mpu.distAHRSMul = dps / tempe
                print(mpu.distAHRSMul)
                statetrigger1 = False
            if (now > starttime + 2) and (statetrigger2 == True):
                mx2 = mpu.mapxAHRS[-1]
                my2 = mpu.mapyAHRS[-1]
                statetrigger2 = False
            if (now > starttime + 3) and (statetrigger3 == True):
                mx3 = mpu.mapxAHRS[-1]
                my3 = mpu.mapyAHRS[-1]
                # mpu.vel = 0
                # mpu.velAHRS = 0
                kendali.enginekill()
                statetrigger3 = False
                #setmaju()
                #setmundur()
            # if (not statetrigger3) and (statetrigger4):
                # mpu.vel = 0
                # mpu.velAHRS = 0
            if (now > starttime + 6) and (statetrigger4 == True):
                mx4 = mpu.mapxAHRS[-1]
                my4 = mpu.mapyAHRS[-1]
                #mpu.vel = 0
                #mpu.velAHRS = 0
                tempe2 = mpu.dispAHRS
                mpu.distAHRSMul = 1
                enginestart()
                statetrigger4 = False
            if (now > starttime + 7) and (statetrigger5 == True):
                mx5 = mpu.mapxAHRS[-1]
                my5 = mpu.mapyAHRS[-1]
                statetrigger5 = False
                tempe = mpu.dispAHRS - tempe2
                mpu.distAHRSMul = dps / tempe
                print(mpu.distAHRSMul)
            if (now > starttime + 8) and (statetrigger6 == True):
                mx6 = mpu.mapxAHRS[-1]
                my6 = mpu.mapyAHRS[-1]
                statetrigger6 = False

            if (kendali.finish) and (statetriggerf == True):
                #Stop Machine
                #shutdownSonar()
                mxf = mpu.mapxAHRS[-1]
                myf = mpu.mapyAHRS[-1]
                statetriggerf = False
                break
            # if (now > starttime + runtime + 0) and (statetriggerf == True):
            #     #Stop Machine
            #     #shutdownSonar()
            #     mxf = mpu.mapxAHRS[-1]
            #     myf = mpu.mapyAHRS[-1]
            #     statetriggerf = False
            # if (not statetriggerf):
            #     mpu.vel = 0
            #     mpu.velAHRS = 0

            #Menghitung speed
            #print('SPD: {}'.format(pwmspd1))
            #======================end of MOTOR DRIVER======================
        #end WHILE
        while(True):
            print('Vel: {:<8.2f} Error:{:<8.3f}'.format( (sum(mpu.arrayVelAHRS) / len(mpu.arrayVelAHRS)), (sum(kendali.pid) / len(kendali.pid)) ) )
            time.sleep(1)
            timt = cet - starttime
            print(f"Runtime: {timt} ; Data Taken: {ce}")

    except KeyboardInterrupt:
        kendali.enginekill()
        #shutdownSonar()
        # End if user hits control c
        #print(posisi)
        #Figure 1 = Tracking robot
        plt.figure(1)
        plt.title('Jejak Robot')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.scatter(0,0,s=1,label='Displacement: {:<8.1f}'.format(mpu.dispAHRS))
        plt.scatter(0,0,s=1,label='Distance: {:<8.1f}'.format(mpu.distFullAHRS))
        plt.scatter(golx[-1],goly[-1],s=50,label='Goal', c='k')
        plt.scatter(mpu.mapx[0],mpu.mapy[0], s=50, label='Start', c='g')
        plt.scatter(mpu.mapxAHRS[-1],mpu.mapyAHRS[-1], s=50, label='Goal Robot', c='red')
        for sct in range(len(kendali.checkPointX)):
            plt.scatter(kendali.checkPointX[sct],kendali.checkPointY[sct],s=25,c='g')
        for sct in range(len(golx)):
            plt.scatter(golx[sct],goly[sct],s=15,c='k')
        # circle1=plt.Circle((golx[-1],golx[-1]),kendali.ae,color='k')
        # plt.gcf().gca().add_artist(circle1)
        #plt.scatter(mpu.mapx[-1],mpu.mapy[-1], s=50, label='Goal', c='r')
        # plt.scatter(mx1,my1, s=25, label='p1', c='gold')
        # plt.scatter(mx2,my2, s=25, label='p2', c='k')
        # plt.scatter(mx3,my3, s=25, label='p3', c='red')
        # plt.scatter(mx4,my4, s=25, label='p4', c='g')
        # plt.scatter(mx5,my5, s=25, label='p5', c='gold')
        # plt.scatter(mx6,my6, s=25, label='p6', c='k')
        # if not statetriggerf:
        #     plt.scatter(mxf,myf, s=25, label='pf', c='r')
        plt.legend(loc='best')
        #plt.plot(mpu.mapx,mpu.mapy, c='b')
        plt.plot(mpu.mapxAHRS,mpu.mapyAHRS, c='y')

        #Figure 2 = Kecepatan Robot
        plt.figure(2)
        plt.title('Kecepatan Robot')
        #plt.plot(mpu.trackingx)
        # plt.plot(mpu.trackingy, label='Raw Accelerometer', c='b')
        plt.plot(mpu.trackingz, label='AHRS', c='y')
        #plt.plot(mpu.arrayaccy)
        plt.legend(loc='best')
        plt.ylabel('Kecepatan')
        #set ke csv
        nptrackingy = np.array(mpu.trackingy)
        nparrayaccy = np.array(mpu.arrayaccy)
        nptrackingx = np.array(mpu.trackingx)
        nparrayaccx = np.array(mpu.arrayaccx)
        nparraydeltat = np.array(mpu.arraydeltat)

        #Figure 3 = Tracking Robot no AHRS
        #plt.figure(3)
        #plt.title('Jejak Robot')
        #plt.xlabel('X, Utara')
        #plt.ylabel('Y, Barat')
        #plt.axis('equal')
        #plt.scatter(mpu.mapx[0],mpu.mapy[0], s=50, label='Start', c='g')
        #plt.scatter(mpu.mapx[-1],mpu.mapy[-1], s=50, label='Goal', c='r')
        #plt.legend(loc='best')
        #plt.plot(mpu.mapx,mpu.mapy, c='b')

        #np.reshape(nptrackingy, (len(nptrackingy),1))
        df = pd.DataFrame({"time" : nparraydeltat, "vel_y" : nptrackingy, "vel_x" : nptrackingx, "acc_y" : nparrayaccy, "acc_x" : nparrayaccx})
        df.to_csv("acc.csv") #velocity y
        #pd.DataFrame(mpu.arrayaccy).to_csv("accy.csv", header=None, index=None) #accelerometer y

        #Figure 4 = Raw + AHRS Tracking
        plt.figure(4)
        plt.title('Akselerasi Robot')
        plt.plot(mpu.arrayaccy, label='Raw Accelerometer', c='b')
        plt.plot(mpu.trackingahrsy, label='AHRS', c='y')
        #plt.plot(mpu.pitched, label='Pitch')
        #plt.plot(mpu.arrayaccy)
        plt.legend(loc='best')
        plt.ylabel('Akselerasi')

        #Figure 4 = Raw + AHRS Tracking
        plt.figure(5)
        plt.title('PID')
        plt.plot(kendali.pid, label='Error, Rata-rata: {:<8.2f}'.format(sum(kendali.pid) / len(kendali.pid)), c='b')
        plt.scatter(0,0.5,s=1)
        plt.scatter(0,-0.5,s=1)
        plt.hlines(y=0, xmin=0, xmax=len(kendali.pid), colors='y', linestyles='--', lw=2)
        plt.legend(loc='best')
        plt.ylabel('Error')


        plt.show()
        print("Closing")
#======================Main Program=======================<<[5]

# Main loop
if __name__ == '__main__':
    main()
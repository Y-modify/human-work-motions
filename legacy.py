# Generated from Actions.ino and ActionFunctions.ino

import math
import time
from humanoid import Humanoid
from threading import Thread
import pybullet

pybullet.connect(pybullet.GUI)

# robot = Humanoid("yamax.urdf", real=True)
robot = Humanoid("yamax.urdf", bullet_client=pybullet)

kI = 0.5

if not robot.is_real:
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_MOUSE_PICKING, 0)
    pybullet.resetDebugVisualizerCamera(0.7 + 1, 75, -15, [0,0,0])

def setServoPulse(idx, deg):
    if idx is not None:
        robot.set_joint_state(idx-8, (deg - 90) / 180 * math.pi)

def delay(ms):
    time.sleep(ms/1000)

def print_c(fmt, *args):
    print(fmt)

# Port definition
if robot.is_real:
    servoA = 8

    servoB = None
    servoC = None
    servoD = None

    servoE = 7
    servoF = None
    servoG = 5

    servoH = 10
    servoI = 9

    servoK = 4
    servoL = 3
    servoM = 2
    servoN = 1
    servoO = 0

    servoQ = 11
    servoR = 12
    servoS = 13
    servoT = 14
    servoU = 15
else:
    servoA = 1

    servoB = 5
    servoC = 6
    servoD = 7

    servoE = 2
    servoF = 3
    servoG = 4

    servoH = 8
    servoI = 9

    servoK = 15
    servoL = 16
    servoM = 17
    servoN = 18
    servoO = 19

    servoQ = 10
    servoR = 11
    servoS = 12
    servoT = 13
    servoU = 14

if robot.is_real:
    stA = 80
    stB = 70
    stC = 80
    stD = 140
    stE = 90
    stF = 70
    stG = 20
    stH = 90
    stI = 120
    stK = 80
    stL = 70
    stM = 105
    stN = 105
    stO = 35
    stQ = 98
    stR = 120
    stS = 100
    stT = 75
    stU = 148
else:
    stA = 90
    stB = 90
    stC = 90
    stD = 90
    stE = 90
    stF = 90
    stG = 90
    stH = 90
    stI = 90
    stK = 90
    stL = 90
    stM = 90
    stN = 90
    stO = 90
    stQ = 90
    stR = 90
    stS = 90
    stT = 90
    stU = 90

# Initialize Servo Driver
def ServoInit():
  delay(100)

# Base Functions
def stand():
  setServoPulse(servoA, stA)

  setServoPulse(servoB, stB)
  setServoPulse(servoC, stC)
  setServoPulse(servoD, stD)

  setServoPulse(servoE, stE)
  setServoPulse(servoF, stF)
  setServoPulse(servoG, stG)

  setServoPulse(servoH, stH)
  setServoPulse(servoI, stI)

  setServoPulse(servoK, stK)
  setServoPulse(servoL, stL)
  setServoPulse(servoM, stM)
  setServoPulse(servoN, stN)
  setServoPulse(servoO, stO)

  setServoPulse(servoQ, stQ)
  setServoPulse(servoR, stR)
  setServoPulse(servoS, stS)
  setServoPulse(servoT, stT)
  setServoPulse(servoU, stU)
#

def bowing():
  for i in range(5+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - i * 9)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + i * 4)
    setServoPulse(servoM, stM)
    setServoPulse(servoN, stN)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - i * 4)
    setServoPulse(servoS, stS)
    setServoPulse(servoT, stT)
    setServoPulse(servoU, stU)

    delay(40)
  #
#

def resetBowing():
  for i in range(5+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 45 + i * 9)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 20 - i * 4)
    setServoPulse(servoM, stM)
    setServoPulse(servoN, stN)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 20 + i * 4)
    setServoPulse(servoS, stS)
    setServoPulse(servoT, stT)
    setServoPulse(servoU, stU)

    delay(40)



def crouch():
  for i in range(5+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - i * 4)
    setServoPulse(servoM, stM + i * 8)
    setServoPulse(servoN, stN + i * 4)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + i * 4)
    setServoPulse(servoS, stS - i * 8)
    setServoPulse(servoT, stT - i * 4)
    setServoPulse(servoU, stU)

    delay(40)



def resetCrouch():
  for i in range(5+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20 + i * 4)
    setServoPulse(servoM, stM + 40 - i * 8)
    setServoPulse(servoN, stN + 20 - i * 4)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20 - i * 4)
    setServoPulse(servoS, stS - 40 + i * 8)
    setServoPulse(servoT, stT - 20 + i * 4)
    setServoPulse(servoU, stU)

    delay(40)



# Walking
# begins
def LEFTwalkBeginUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - i * 2.5)
    setServoPulse(servoI, stI - i * kI) # モル生えるwww

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + i * 1.25)

    delay(frame)



def LEFTwalkBeginFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - kI*4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40 + i * 5)
    setServoPulse(servoT, stT - 20 + i * 5)
    setServoPulse(servoU, stU + 10)
    delay(frame)



def LEFTwalkBeginDown(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 2.5)
    setServoPulse(servoI, stI - kI * 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20 - i * 0.5)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20 + i + 0.5)
    setServoPulse(servoS, stS - 20 + i * 2.5)
    setServoPulse(servoT, stT + i * 3)
    setServoPulse(servoU, stU + 10 - i * 1.25)
    delay(frame)



def RIGHTwalkBeginUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + i * 2.5)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - i * 1.25)

    delay(frame)


def RIGHTwalkBeginFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40 - i * 5)
    setServoPulse(servoN, stN + 20 - i * 5)
    setServoPulse(servoO, stO - 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 10)
    delay(frame)



def RIGHTwalkBeginDown(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20 - i * 2.5)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 20 - i * 2.5)
    setServoPulse(servoN, stN - i * 3)
    setServoPulse(servoO, stO - 10 + i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 10 + i * 1.25)
    delay(frame)



def LEFTbackBeginFoward(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - 20 + i * 2)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20 - i * 3)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20 - i * 2)
    setServoPulse(servoU, stU + 10)
    delay(frame)



def LEFTbackBeginDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 4)
    setServoPulse(servoS, stS - 40 + i * 4)
    setServoPulse(servoT, stT - 36 + i * 2)
    setServoPulse(servoU, stU + 10 - i * 2.5)
    delay(frame)


# walking
def leftEndFoward(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - kI*4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + i * 5)
    setServoPulse(servoN, stN - 20 + i * 5)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10)

    delay(frame)



def leftEnd(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - kI*4 + i * 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24 + i)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24 - i)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10 - i * 2.5)

    delay(frame)



def leftUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - i * 5)
    setServoPulse(servoI, stI - kI * 4 - i * 2)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM)
    setServoPulse(servoN, stN - 24 + i)
    setServoPulse(servoO, stO + i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + i * 2.5)

    delay(frame)



def rightUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + i * 5)
    setServoPulse(servoI, stI - kI*6 - i * 2)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS)
    setServoPulse(servoT, stT + 24 - i)
    setServoPulse(servoU, stU - i * 2.5)

    delay(frame)



def leftFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - kI * 6)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + i * 5)
    setServoPulse(servoN, stN - 20 + i * 4.25)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - 40 + i * 5)
    setServoPulse(servoT, stT - 20 + i * 5)
    setServoPulse(servoU, stU + 10)

    delay(frame)



def rightFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20)
    setServoPulse(servoI, stI - kI*6)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + 40 - i * 5)
    setServoPulse(servoN, stN + 20 - i * 5)
    setServoPulse(servoO, stO - 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - i * 5)
    setServoPulse(servoT, stT + 20 - i * 4.25)
    setServoPulse(servoU, stU - 10)

    delay(frame)



def leftDown(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 2.5)
    setServoPulse(servoI, stI - kI*6 + i)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + 20 + i * 2.5)
    setServoPulse(servoN, stN - 3 + i * 2.875)
    setServoPulse(servoO, stO + 10 - i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - 20 + i * 2.5)
    setServoPulse(servoT, stT + i * 3)
    setServoPulse(servoU, stU + 10 - i * 1.25)

    delay(frame)



def rightDown(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20 - i * 2.5)
    setServoPulse(servoI, stI - kI*6 + i)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24)
    setServoPulse(servoM, stM + 20 - i * 2.5)
    setServoPulse(servoN, stN - i * 3)
    setServoPulse(servoO, stO - 10 + i * 1.25)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24)
    setServoPulse(servoS, stS - 20 - i * 2.5)
    setServoPulse(servoT, stT + 3 - i * 2.875)
    setServoPulse(servoU, stU - 10 + i * 1.25)

    delay(frame)



# back
def backLEFTEndFoward(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 4 - i * 3)
    setServoPulse(servoM, stM + 24 + i * 2)
    setServoPulse(servoN, stN + 28 - i)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10)
    delay(frame)



def backLEFTEnd(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - 4 + i)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + -20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10 - i * 2.5)
    delay(frame)



def backLEFTUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - i * 5)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 4)
    setServoPulse(servoM, stM + 24)
    setServoPulse(servoN, stN + 28)
    setServoPulse(servoO, stO + i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + i * 2.5)
    delay(frame)



def backLEFTFoward(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 4 - i * 3)
    setServoPulse(servoM, stM + 24 + i * 2)
    setServoPulse(servoN, stN + 28 - i)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20 - i * 3)
    setServoPulse(servoS, stS - 40 + i * 2)
    setServoPulse(servoT, stT - 20 - i)
    setServoPulse(servoU, stU + 10)
    delay(frame)



def backLEFTDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 4)
    setServoPulse(servoS, stS - 24)
    setServoPulse(servoT, stT - 28)
    setServoPulse(servoU, stU + 10 - i * 2.5)
    delay(frame)



def backRIGHTUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + i * 5)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 4)
    setServoPulse(servoS, stS - 24)
    setServoPulse(servoT, stT - 28)
    setServoPulse(servoU, stU - i * 2.5)
    delay(frame)



def backRIGHTFoward(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20 + i * 3)
    setServoPulse(servoM, stM + 40 - i * 2)
    setServoPulse(servoN, stN + 20 + i)
    setServoPulse(servoO, stO - 10)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 4 + i * 3)
    setServoPulse(servoS, stS - 24 - i * 2)
    setServoPulse(servoT, stT - 28 + i)
    setServoPulse(servoU, stU - 10)
    delay(frame)



def backRIGHTDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20 - i * 5)
    setServoPulse(servoI, stI - 4)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 4)
    setServoPulse(servoM, stM + 24)
    setServoPulse(servoN, stN + 28)
    setServoPulse(servoO, stO - 10 + i * 2.5)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 10 + i * 2.5)
    delay(frame)



def backRIGHTToCrouch(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 4 - i * 3)
    setServoPulse(servoM, stM + 24 + i * 2)
    setServoPulse(servoN, stN + 28 - i)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU)
    delay(frame)



# turn
def turnRightLEFTToCrouch(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - i * 5)
    setServoPulse(servoT, stT + 24 - i * 5.5)
    setServoPulse(servoU, stU)
    delay(frame)



def turnLeftRIGHTToCrouch(frame):
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 10)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + i * 5)
    setServoPulse(servoN, stN - 24 + i * 5.5)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU)
    delay(frame)



# crab
def RIGHTrightCrabFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - i * 2)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - 10)

    setServoPulse(servoQ, stQ + i * 2)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 10)

    delay(frame)



def RIGHTrightCrabDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20 - i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - 10 + i * 4)

    setServoPulse(servoQ, stQ + 8)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 10 + i)

    delay(frame)



def RIGHTleftCrabUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 6 - i * 2.5)

    setServoPulse(servoQ, stQ + 8)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 6 + i * 2)

    delay(frame)



def RIGHTleftCrabFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8 + i * 2)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - 4)

    setServoPulse(servoQ, stQ + 8 - i * 2)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 4)

    delay(frame)



def RIGHTleftCrabDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - 4 + i)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 4 - i)

    delay(frame)



def LEFTleftCrabFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - i * 2)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10)

    setServoPulse(servoQ, stQ + i * 2)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10)

    delay(frame)



def LEFTleftCrabDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH - 20 + i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 10 - i * 4)

    setServoPulse(servoQ, stQ + 8)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 10 - i)

    delay(frame)



def LEFTrightCrabUp(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO - 6 + i * 2.5)

    setServoPulse(servoQ, stQ + 8)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU + 6 - i * 2)

    delay(frame)



def LEFTrightCrabFoward(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK - 8 + i * 2)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 4)

    setServoPulse(servoQ, stQ + 8 - i * 2)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 4)

    delay(frame)



def LEFTrightCrabDown(frame):
  for i in range(4+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH + 20 - i * 5)
    setServoPulse(servoI, stI - 15)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 20)
    setServoPulse(servoM, stM + 40)
    setServoPulse(servoN, stN + 20)
    setServoPulse(servoO, stO + 4 - i)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 20)
    setServoPulse(servoS, stS - 40)
    setServoPulse(servoT, stT - 20)
    setServoPulse(servoU, stU - 4 + i)

    delay(frame)



# dogeza
def seiza1():
  for i in range(10+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - i * 3)
    setServoPulse(servoM, stM + i * 7)
    setServoPulse(servoN, stN + i * 4)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + i * 3)
    setServoPulse(servoS, stS - i * 7)
    setServoPulse(servoT, stT - i * 4)
    setServoPulse(servoU, stU)

    delay(40)


# sit
def sitdown1():
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - i * 4.5)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + i * 2)
    setServoPulse(servoM, stM)
    setServoPulse(servoN, stN)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - i * 2)
    setServoPulse(servoS, stS)
    setServoPulse(servoT, stT)
    setServoPulse(servoU, stU)

    delay(50)



def sitdown2():
  for i in range(16+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 36 - i * 2.25)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 16 + i * 0.5)
    setServoPulse(servoM, stM + i)
    setServoPulse(servoN, stN - i * 1.5)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 16 - i * 0.5)
    setServoPulse(servoS, stS - i)
    setServoPulse(servoT, stT + i * 1.5)
    setServoPulse(servoU, stU)

    delay(50)



def situp1():
  for i in range(8+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 72)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL + 24 - i * 6)
    setServoPulse(servoM, stM + 16 - i)
    setServoPulse(servoN, stN - 24)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR - 24 + i * 6)
    setServoPulse(servoS, stS - 16 + i)
    setServoPulse(servoT, stT + 24)
    setServoPulse(servoU, stU)

    delay(60)



def situp2():
  for i in range(16+1):
    setServoPulse(servoA, stA)

    setServoPulse(servoB, stB)
    setServoPulse(servoC, stC)
    setServoPulse(servoD, stD)

    setServoPulse(servoE, stE)
    setServoPulse(servoF, stF)
    setServoPulse(servoG, stG)

    setServoPulse(servoH, stH)
    setServoPulse(servoI, stI - 72 + i * 4.5)

    setServoPulse(servoK, stK)
    setServoPulse(servoL, stL - 24 + i * 1.5)
    setServoPulse(servoM, stM + 8 - i * 0.5)
    setServoPulse(servoN, stN - 24 + i * 1.5)
    setServoPulse(servoO, stO)

    setServoPulse(servoQ, stQ)
    setServoPulse(servoR, stR + 24 - i * 1.5)
    setServoPulse(servoS, stS - 8 + i * 0.5)
    setServoPulse(servoT, stT + 24 - i * 1.5)
    setServoPulse(servoU, stU)

    delay(50)

def walk(times, delaytime):

  if (delaytime < 20):
    delaytime = 20

  # print_c(f"Walking {times} steps with {delaytime}ms\n")

  if (times <= 2):
    times = times
  else:
    times = times - 2

  crouch()
  delay(500)
  LEFTwalkBeginUp(delaytime + 10)
  LEFTwalkBeginFoward(delaytime + 10)
  LEFTwalkBeginDown(delaytime + 10)
  for i in range(0,times,2):

    if (i != 0):

      leftUp(delaytime)
      leftFoward(delaytime)
      leftDown(delaytime)

    rightUp(delaytime)
    rightFoward(delaytime)
    rightDown(delaytime)

  leftUp(delaytime + 10)
  leftEndFoward(delaytime + 20)
  leftEnd(delaytime + 30)

  delay(500)
  resetCrouch()







def back(times, delaytime):

  if (delaytime < 20):
    delaytime = 20

  print_c("Walking %d steps with %dms\n", times, delaytime)

  if (times <= 2):
    times = times
  else:
    times = times - 2

  crouch()
  delay(500)
  LEFTwalkBeginUp(delaytime)
  LEFTbackBeginFoward(delaytime)
  LEFTbackBeginDown(delaytime)

  for i in range(0,times,2):

    if (i != 0):

      backLEFTUp(delaytime)
      backLEFTFoward(delaytime)
      backLEFTDown(delaytime)

    backRIGHTUp(delaytime)
    backRIGHTFoward(delaytime)
    backRIGHTDown(delaytime)

  backLEFTUp(delaytime)
  backLEFTEndFoward(delaytime)
  backLEFTEnd(delaytime)


  delay(500)
  resetCrouch()








def crabWalkingRight(times, delaytime):

  if (delaytime < 20):
    delaytime = 20

  print_c("CrabWalkingRight %d steps with %dms\n", times, delaytime)

  crouch()
  delay(500)
  for i in range(times):

    RIGHTwalkBeginUp(delaytime)
    RIGHTrightCrabFoward(delaytime)
    RIGHTrightCrabDown(delaytime)

    RIGHTleftCrabUp(delaytime)
    RIGHTleftCrabFoward(delaytime)
    RIGHTleftCrabDown(delaytime)

  delay(500)
  resetCrouch()
  delay(2000)







def crabWalkingLeft(times, delaytime):

  if (delaytime < 20):
    delaytime = 20

  print_c("CrabWalkingRight %d steps with %dms\n", times, delaytime)

  crouch()
  delay(500)
  for i in range(times):

    LEFTwalkBeginUp(delaytime)
    LEFTleftCrabFoward(delaytime)
    LEFTleftCrabDown(delaytime)

    LEFTrightCrabUp(delaytime)
    LEFTrightCrabFoward(delaytime)
    LEFTrightCrabDown(delaytime)

  delay(500)
  resetCrouch()
  delay(2000)







def turnright(times, delaytime):

  print_c("Turning right %d times with %dms\n", times, delaytime)
  if (delaytime < 20):
    delaytime = 20

  crouch()
  delay(500)
  i = 0
  for i in range(times):

    LEFTwalkBeginUp(delaytime)
    LEFTwalkBeginFoward(delaytime)
    LEFTwalkBeginDown(delaytime)
    turnRightLEFTToCrouch(delaytime)

  delay(500)
  resetCrouch()







def turnleft(times, delaytime):

  print_c("Turning left %d times with %dms\n", times, delaytime)
  if (delaytime < 20):
    delaytime = 20

  crouch()
  delay(500)
  i = 0
  for i in range(times):

    RIGHTwalkBeginUp(delaytime)
    RIGHTwalkBeginFoward(delaytime)
    RIGHTwalkBeginDown(delaytime)
    turnLeftRIGHTToCrouch(delaytime)

  delay(500)
  resetCrouch()





def automaDogeza(delaytime):

  seiza1()
  delay(delaytime)






def no(times):

  print_c("Denying %d times\n", times)

  for i in range(times):

    setServoPulse(servoA, stA - 60)
    delay(200)
    setServoPulse(servoA, stA + 60)
    delay(200)

  setServoPulse(servoA, stA)






def bow(delaytime):

  print_c("Bowing %dms\n", delaytime)
  bowing()
  delay(delaytime)
  resetBowing()







def bye(times, dirr):

  print_c("Bye %d times ", times)

  stand()

  if (dirr == 0):

    print_c("with right hand\n", times)
    setServoPulse(servoE, stE + 80)
    setServoPulse(servoF, stF - 70)
    for i in range(times):

      tim = 20
      for j in range(8):

        setServoPulse(servoG, stG + 80 - j * 10)
        delay(tim)

      for j in range(8):

        setServoPulse(servoG, stG + j * 10)
        delay(tim)




  if (dirr == 1):
    print_c("with left hand\n", times)
    setServoPulse(servoB, stB - 50)
    setServoPulse(servoC, stC + 80)
    for i in range(times):

      tim = 20
      for j in range(8):

        setServoPulse(servoD, stD - 80 + j * 10)
        delay(tim)

      for j in range(8):

        setServoPulse(servoD, stD - j * 10)
        delay(tim)






  if (dirr == 2):

    print_c("with both hand\n", times)

    setServoPulse(servoB, stB - 50)
    setServoPulse(servoC, stC + 80)
    setServoPulse(servoE, stE + 50)
    setServoPulse(servoF, stF - 60)
    for i in range(times):

      tim = 20
      for j in range(8):

        setServoPulse(servoG, stG + 80 - j * 10)
        setServoPulse(servoD, stD - 80 + j * 10)
        delay(tim)

      for j in range(8):

        setServoPulse(servoG, stG + j * 10)
        setServoPulse(servoD, stD - j * 10)
        delay(tim)











def nadenade(times, dirr):

  print_c("Nadenade %d times\n", times)

  stand()

  if (dirr):

    print_c("with left hand\n", times)
    setServoPulse(servoC, stC + 80)

    for i in range(times):

      tim = 20
      for j in range(8):

        setServoPulse(servoD, stD - 80 + j * 10)
        delay(tim)

      for j in range(8):

        setServoPulse(servoD, stD - j * 10)
        delay(tim)




  if (not dirr):

    print_c("with right hand\n", times)
    setServoPulse(servoF, stF - 60)
    for i in range(times):

      tim = 20
      for j in range(8):

        setServoPulse(servoG, stG + 80 - j * 10)
        delay(tim)

      for j in range(8):

        setServoPulse(servoG, stG + j * 10)
        delay(tim)






def sitDown():

  sitdown1()
  sitdown2()


def sitUp():

  situp1()
  delay(1000)
  situp2()


stand()

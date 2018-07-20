# Generated from Actions.ino and ActionFunctions.ino

import math
import time
from threading import Thread


class Motions(object):
    def __init__(self, robot):
        self.kI = 0.5
        self.robot = robot

        # Port definition
        if self.robot.is_real:
            self.servoA = 8

            self.servoB = None
            self.servoC = None
            self.servoD = None

            self.servoE = 7
            self.servoF = None
            self.servoG = 5

            self.servoH = 10
            self.servoI = 9

            self.servoK = 4
            self.servoL = 3
            self.servoM = 2
            self.servoN = 1
            self.servoO = 0

            self.servoQ = 11
            self.servoR = 12
            self.servoS = 13
            self.servoT = 14
            self.servoU = 15
        else:
            self.servoA = 1

            self.servoB = 5
            self.servoC = 6
            self.servoD = 7

            self.servoE = 2
            self.servoF = 3
            self.servoG = 4

            self.servoH = 8
            self.servoI = 9

            self.servoK = 15
            self.servoL = 16
            self.servoM = 17
            self.servoN = 18
            self.servoO = 19

            self.servoQ = 10
            self.servoR = 11
            self.servoS = 12
            self.servoT = 13
            self.servoU = 14

        if self.robot.is_real:
            self.stA = 80
            self.stB = 70
            self.stC = 80
            self.stD = 140
            self.stE = 90
            self.stF = 70
            self.stG = 20
            self.stH = 90
            self.stI = 120
            self.stK = 80
            self.stL = 70
            self.stM = 105
            self.stN = 105
            self.stO = 35
            self.stQ = 98
            self.stR = 120
            self.stS = 100
            self.stT = 75
            self.stU = 148
        else:
            self.stA = 90
            self.stB = 90
            self.stC = 90
            self.stD = 90
            self.stE = 90
            self.stF = 90
            self.stG = 90
            self.stH = 90
            self.stI = 90
            self.stK = 90
            self.stL = 90
            self.stM = 90
            self.stN = 90
            self.stO = 90
            self.stQ = 90
            self.stR = 90
            self.stS = 90
            self.stT = 90
            self.stU = 90

    def setServoPulse(self, idx, deg):
        if idx is not None:
            self.robot.set_joint_state(idx-8, (deg - 90) / 180 * math.pi)

    def delay(self, ms):
        time.sleep(ms/1000)

    # Initialize Servo Driver
    def ServoInit(self):
        self.delay(100)

    # Base Functions
    def stand(self):
        self.setServoPulse(self.servoA, self.stA)

        self.setServoPulse(self.servoB, self.stB)
        self.setServoPulse(self.servoC, self.stC)
        self.setServoPulse(self.servoD, self.stD)

        self.setServoPulse(self.servoE, self.stE)
        self.setServoPulse(self.servoF, self.stF)
        self.setServoPulse(self.servoG, self.stG)

        self.setServoPulse(self.servoH, self.stH)
        self.setServoPulse(self.servoI, self.stI)

        self.setServoPulse(self.servoK, self.stK)
        self.setServoPulse(self.servoL, self.stL)
        self.setServoPulse(self.servoM, self.stM)
        self.setServoPulse(self.servoN, self.stN)
        self.setServoPulse(self.servoO, self.stO)

        self.setServoPulse(self.servoQ, self.stQ)
        self.setServoPulse(self.servoR, self.stR)
        self.setServoPulse(self.servoS, self.stS)
        self.setServoPulse(self.servoT, self.stT)
        self.setServoPulse(self.servoU, self.stU)
    #

    def bowing(self):
        for i in range(5+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - i * 9)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + i * 4)
            self.setServoPulse(self.servoM, self.stM)
            self.setServoPulse(self.servoN, self.stN)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - i * 4)
            self.setServoPulse(self.servoS, self.stS)
            self.setServoPulse(self.servoT, self.stT)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(40)

    def resetBowing(self):
        for i in range(5+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 45 + i * 9)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 20 - i * 4)
            self.setServoPulse(self.servoM, self.stM)
            self.setServoPulse(self.servoN, self.stN)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 20 + i * 4)
            self.setServoPulse(self.servoS, self.stS)
            self.setServoPulse(self.servoT, self.stT)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(40)

    def crouch(self):
        for i in range(5+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - i * 4)
            self.setServoPulse(self.servoM, self.stM + i * 8)
            self.setServoPulse(self.servoN, self.stN + i * 4)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + i * 4)
            self.setServoPulse(self.servoS, self.stS - i * 8)
            self.setServoPulse(self.servoT, self.stT - i * 4)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(40)

    def resetCrouch(self):
        for i in range(5+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20 + i * 4)
            self.setServoPulse(self.servoM, self.stM + 40 - i * 8)
            self.setServoPulse(self.servoN, self.stN + 20 - i * 4)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20 - i * 4)
            self.setServoPulse(self.servoS, self.stS - 40 + i * 8)
            self.setServoPulse(self.servoT, self.stT - 20 + i * 4)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(40)

    # Walking
    # begins

    def LEFTwalkBeginUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - i * 2.5)
            self.setServoPulse(self.servoI, self.stI - i * kI)  # モル生えるwww

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + i * 1.25)

            self.delay(frame)

    def LEFTwalkBeginFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - kI*4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40 + i * 5)
            self.setServoPulse(self.servoT, self.stT - 20 + i * 5)
            self.setServoPulse(self.servoU, self.stU + 10)
            self.delay(frame)

    def LEFTwalkBeginDown(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 2.5)
            self.setServoPulse(self.servoI, self.stI - kI * 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20 - i * 0.5)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20 + i + 0.5)
            self.setServoPulse(self.servoS, self.stS - 20 + i * 2.5)
            self.setServoPulse(self.servoT, self.stT + i * 3)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 1.25)
            self.delay(frame)

    def RIGHTwalkBeginUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + i * 2.5)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - i * 1.25)

            self.delay(frame)

    def RIGHTwalkBeginFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40 - i * 5)
            self.setServoPulse(self.servoN, self.stN + 20 - i * 5)
            self.setServoPulse(self.servoO, self.stO - 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 10)
            self.delay(frame)

    def RIGHTwalkBeginDown(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20 - i * 2.5)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 20 - i * 2.5)
            self.setServoPulse(self.servoN, self.stN - i * 3)
            self.setServoPulse(self.servoO, self.stO - 10 + i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 10 + i * 1.25)
            self.delay(frame)

    def LEFTbackBeginFoward(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - 20 + i * 2)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20 - i * 3)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20 - i * 2)
            self.setServoPulse(self.servoU, self.stU + 10)
            self.delay(frame)

    def LEFTbackBeginDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 4)
            self.setServoPulse(self.servoS, self.stS - 40 + i * 4)
            self.setServoPulse(self.servoT, self.stT - 36 + i * 2)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 2.5)
            self.delay(frame)

    # walking

    def leftEndFoward(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - kI*4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + i * 5)
            self.setServoPulse(self.servoN, self.stN - 20 + i * 5)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10)

            self.delay(frame)

    def leftEnd(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - kI*4 + i * 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24 + i)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24 - i)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 2.5)

            self.delay(frame)

    def leftUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - i * 5)
            self.setServoPulse(self.servoI, self.stI - kI * 4 - i * 2)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM)
            self.setServoPulse(self.servoN, self.stN - 24 + i)
            self.setServoPulse(self.servoO, self.stO + i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + i * 2.5)

            self.delay(frame)

    def rightUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + i * 5)
            self.setServoPulse(self.servoI, self.stI - kI*6 - i * 2)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS)
            self.setServoPulse(self.servoT, self.stT + 24 - i)
            self.setServoPulse(self.servoU, self.stU - i * 2.5)

            self.delay(frame)

    def leftFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - kI * 6)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + i * 5)
            self.setServoPulse(self.servoN, self.stN - 20 + i * 4.25)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - 40 + i * 5)
            self.setServoPulse(self.servoT, self.stT - 20 + i * 5)
            self.setServoPulse(self.servoU, self.stU + 10)

            self.delay(frame)

    def rightFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20)
            self.setServoPulse(self.servoI, self.stI - kI*6)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + 40 - i * 5)
            self.setServoPulse(self.servoN, self.stN + 20 - i * 5)
            self.setServoPulse(self.servoO, self.stO - 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - i * 5)
            self.setServoPulse(self.servoT, self.stT + 20 - i * 4.25)
            self.setServoPulse(self.servoU, self.stU - 10)

            self.delay(frame)

    def leftDown(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 2.5)
            self.setServoPulse(self.servoI, self.stI - kI*6 + i)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + 20 + i * 2.5)
            self.setServoPulse(self.servoN, self.stN - 3 + i * 2.875)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - 20 + i * 2.5)
            self.setServoPulse(self.servoT, self.stT + i * 3)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 1.25)

            self.delay(frame)

    def rightDown(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20 - i * 2.5)
            self.setServoPulse(self.servoI, self.stI - kI*6 + i)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24)
            self.setServoPulse(self.servoM, self.stM + 20 - i * 2.5)
            self.setServoPulse(self.servoN, self.stN - i * 3)
            self.setServoPulse(self.servoO, self.stO - 10 + i * 1.25)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24)
            self.setServoPulse(self.servoS, self.stS - 20 - i * 2.5)
            self.setServoPulse(self.servoT, self.stT + 3 - i * 2.875)
            self.setServoPulse(self.servoU, self.stU - 10 + i * 1.25)

            self.delay(frame)

    # back

    def backLEFTEndFoward(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 4 - i * 3)
            self.setServoPulse(self.servoM, self.stM + 24 + i * 2)
            self.setServoPulse(self.servoN, self.stN + 28 - i)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10)
            self.delay(frame)

    def backLEFTEnd(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - 4 + i)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + -20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 2.5)
            self.delay(frame)

    def backLEFTUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - i * 5)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 4)
            self.setServoPulse(self.servoM, self.stM + 24)
            self.setServoPulse(self.servoN, self.stN + 28)
            self.setServoPulse(self.servoO, self.stO + i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + i * 2.5)
            self.delay(frame)

    def backLEFTFoward(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 4 - i * 3)
            self.setServoPulse(self.servoM, self.stM + 24 + i * 2)
            self.setServoPulse(self.servoN, self.stN + 28 - i)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20 - i * 3)
            self.setServoPulse(self.servoS, self.stS - 40 + i * 2)
            self.setServoPulse(self.servoT, self.stT - 20 - i)
            self.setServoPulse(self.servoU, self.stU + 10)
            self.delay(frame)

    def backLEFTDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 4)
            self.setServoPulse(self.servoS, self.stS - 24)
            self.setServoPulse(self.servoT, self.stT - 28)
            self.setServoPulse(self.servoU, self.stU + 10 - i * 2.5)
            self.delay(frame)

    def backRIGHTUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + i * 5)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 4)
            self.setServoPulse(self.servoS, self.stS - 24)
            self.setServoPulse(self.servoT, self.stT - 28)
            self.setServoPulse(self.servoU, self.stU - i * 2.5)
            self.delay(frame)

    def backRIGHTFoward(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20 + i * 3)
            self.setServoPulse(self.servoM, self.stM + 40 - i * 2)
            self.setServoPulse(self.servoN, self.stN + 20 + i)
            self.setServoPulse(self.servoO, self.stO - 10)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 4 + i * 3)
            self.setServoPulse(self.servoS, self.stS - 24 - i * 2)
            self.setServoPulse(self.servoT, self.stT - 28 + i)
            self.setServoPulse(self.servoU, self.stU - 10)
            self.delay(frame)

    def backRIGHTDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20 - i * 5)
            self.setServoPulse(self.servoI, self.stI - 4)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 4)
            self.setServoPulse(self.servoM, self.stM + 24)
            self.setServoPulse(self.servoN, self.stN + 28)
            self.setServoPulse(self.servoO, self.stO - 10 + i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 10 + i * 2.5)
            self.delay(frame)

    def backRIGHTToCrouch(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 4 - i * 3)
            self.setServoPulse(self.servoM, self.stM + 24 + i * 2)
            self.setServoPulse(self.servoN, self.stN + 28 - i)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU)
            self.delay(frame)

    # turn

    def turnRightLEFTToCrouch(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - i * 5)
            self.setServoPulse(self.servoT, self.stT + 24 - i * 5.5)
            self.setServoPulse(self.servoU, self.stU)
            self.delay(frame)

    def turnLeftRIGHTToCrouch(self, frame):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 10)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + i * 5)
            self.setServoPulse(self.servoN, self.stN - 24 + i * 5.5)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU)
            self.delay(frame)

    # crab

    def RIGHTrightCrabFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - i * 2)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - 10)

            self.setServoPulse(self.servoQ, self.stQ + i * 2)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 10)

            self.delay(frame)

    def RIGHTrightCrabDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20 - i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - 10 + i * 4)

            self.setServoPulse(self.servoQ, self.stQ + 8)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 10 + i)

            self.delay(frame)

    def RIGHTleftCrabUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 6 - i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ + 8)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 6 + i * 2)

            self.delay(frame)

    def RIGHTleftCrabFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8 + i * 2)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - 4)

            self.setServoPulse(self.servoQ, self.stQ + 8 - i * 2)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 4)

            self.delay(frame)

    def RIGHTleftCrabDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - 4 + i)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 4 - i)

            self.delay(frame)

    def LEFTleftCrabFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - i * 2)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10)

            self.setServoPulse(self.servoQ, self.stQ + i * 2)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10)

            self.delay(frame)

    def LEFTleftCrabDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH - 20 + i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 10 - i * 4)

            self.setServoPulse(self.servoQ, self.stQ + 8)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 10 - i)

            self.delay(frame)

    def LEFTrightCrabUp(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO - 6 + i * 2.5)

            self.setServoPulse(self.servoQ, self.stQ + 8)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU + 6 - i * 2)

            self.delay(frame)

    def LEFTrightCrabFoward(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK - 8 + i * 2)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 4)

            self.setServoPulse(self.servoQ, self.stQ + 8 - i * 2)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 4)

            self.delay(frame)

    def LEFTrightCrabDown(self, frame):
        for i in range(4+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH + 20 - i * 5)
            self.setServoPulse(self.servoI, self.stI - 15)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 20)
            self.setServoPulse(self.servoM, self.stM + 40)
            self.setServoPulse(self.servoN, self.stN + 20)
            self.setServoPulse(self.servoO, self.stO + 4 - i)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 20)
            self.setServoPulse(self.servoS, self.stS - 40)
            self.setServoPulse(self.servoT, self.stT - 20)
            self.setServoPulse(self.servoU, self.stU - 4 + i)

            self.delay(frame)

    # dogeza

    def seiza1(self):
        for i in range(10+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - i * 3)
            self.setServoPulse(self.servoM, self.stM + i * 7)
            self.setServoPulse(self.servoN, self.stN + i * 4)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + i * 3)
            self.setServoPulse(self.servoS, self.stS - i * 7)
            self.setServoPulse(self.servoT, self.stT - i * 4)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(40)

    # sit

    def sitdown1(self):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - i * 4.5)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + i * 2)
            self.setServoPulse(self.servoM, self.stM)
            self.setServoPulse(self.servoN, self.stN)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - i * 2)
            self.setServoPulse(self.servoS, self.stS)
            self.setServoPulse(self.servoT, self.stT)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(50)

    def sitdown2(self):
        for i in range(16+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 36 - i * 2.25)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 16 + i * 0.5)
            self.setServoPulse(self.servoM, self.stM + i)
            self.setServoPulse(self.servoN, self.stN - i * 1.5)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 16 - i * 0.5)
            self.setServoPulse(self.servoS, self.stS - i)
            self.setServoPulse(self.servoT, self.stT + i * 1.5)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(50)

    def situp1(self):
        for i in range(8+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 72)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL + 24 - i * 6)
            self.setServoPulse(self.servoM, self.stM + 16 - i)
            self.setServoPulse(self.servoN, self.stN - 24)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR - 24 + i * 6)
            self.setServoPulse(self.servoS, self.stS - 16 + i)
            self.setServoPulse(self.servoT, self.stT + 24)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(60)

    def situp2(self):
        for i in range(16+1):
            self.setServoPulse(self.servoA, self.stA)

            self.setServoPulse(self.servoB, self.stB)
            self.setServoPulse(self.servoC, self.stC)
            self.setServoPulse(self.servoD, self.stD)

            self.setServoPulse(self.servoE, self.stE)
            self.setServoPulse(self.servoF, self.stF)
            self.setServoPulse(self.servoG, self.stG)

            self.setServoPulse(self.servoH, self.stH)
            self.setServoPulse(self.servoI, self.stI - 72 + i * 4.5)

            self.setServoPulse(self.servoK, self.stK)
            self.setServoPulse(self.servoL, self.stL - 24 + i * 1.5)
            self.setServoPulse(self.servoM, self.stM + 8 - i * 0.5)
            self.setServoPulse(self.servoN, self.stN - 24 + i * 1.5)
            self.setServoPulse(self.servoO, self.stO)

            self.setServoPulse(self.servoQ, self.stQ)
            self.setServoPulse(self.servoR, self.stR + 24 - i * 1.5)
            self.setServoPulse(self.servoS, self.stS - 8 + i * 0.5)
            self.setServoPulse(self.servoT, self.stT + 24 - i * 1.5)
            self.setServoPulse(self.servoU, self.stU)

            self.delay(50)

    def walk(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        if (times <= 2):
            times = times
        else:
            times = times - 2

        self.crouch()
        self.delay(500)
        self.LEFTwalkBeginUp(delaytime + 10)
        self.LEFTwalkBeginFoward(delaytime + 10)
        self.LEFTwalkBeginDown(delaytime + 10)
        for i in range(0, times, 2):

            if (i != 0):

                self.leftUp(delaytime)
                self.leftFoward(delaytime)
                self.leftDown(delaytime)

            self.rightUp(delaytime)
            self.rightFoward(delaytime)
            self.rightDown(delaytime)

        self.leftUp(delaytime + 10)
        self.leftEndFoward(delaytime + 20)
        self.leftEnd(delaytime + 30)

        self.delay(500)
        self.resetCrouch()

    def back(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        if (times <= 2):
            times = times
        else:
            times = times - 2

        self.crouch()
        self.delay(500)
        self.LEFTwalkBeginUp(delaytime)
        self.LEFTbackBeginFoward(delaytime)
        self.LEFTbackBeginDown(delaytime)

        for i in range(0, times, 2):

            if (i != 0):

                self.backLEFTUp(delaytime)
                self.backLEFTFoward(delaytime)
                self.backLEFTDown(delaytime)

            self.backRIGHTUp(delaytime)
            self.backRIGHTFoward(delaytime)
            self.backRIGHTDown(delaytime)

        self.backLEFTUp(delaytime)
        self.backLEFTEndFoward(delaytime)
        self.backLEFTEnd(delaytime)

        self.delay(500)
        self.resetCrouch()

    def crabWalkingRight(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        self.crouch()
        self.delay(500)
        for i in range(times):

            self.RIGHTwalkBeginUp(delaytime)
            self.RIGHTrightCrabFoward(delaytime)
            self.RIGHTrightCrabDown(delaytime)

            self.RIGHTleftCrabUp(delaytime)
            self.RIGHTleftCrabFoward(delaytime)
            self.RIGHTleftCrabDown(delaytime)

        self.delay(500)
        self.resetCrouch()
        self.delay(2000)

    def crabWalkingLeft(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        self.crouch()
        self.delay(500)
        for i in range(times):

            self.LEFTwalkBeginUp(delaytime)
            self.LEFTleftCrabFoward(delaytime)
            self.LEFTleftCrabDown(delaytime)

            self.LEFTrightCrabUp(delaytime)
            self.LEFTrightCrabFoward(delaytime)
            self.LEFTrightCrabDown(delaytime)

        self.delay(500)
        self.resetCrouch()
        self.delay(2000)

    def turnright(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        self.crouch()
        self.delay(500)
        i = 0
        for i in range(times):

            self.LEFTwalkBeginUp(delaytime)
            self.LEFTwalkBeginFoward(delaytime)
            self.LEFTwalkBeginDown(delaytime)
            self.turnRightLEFTToCrouch(delaytime)

        self.delay(500)
        self.resetCrouch()

    def turnleft(self, times, delaytime):

        if (delaytime < 20):
            delaytime = 20

        self.crouch()
        self.delay(500)
        i = 0
        for i in range(times):

            self.RIGHTwalkBeginUp(delaytime)
            self.RIGHTwalkBeginFoward(delaytime)
            self.RIGHTwalkBeginDown(delaytime)
            self.turnLeftRIGHTToCrouch(delaytime)

        self.delay(500)
        self.resetCrouch()

    def automaDogeza(self, delaytime):

        self.seiza1()
        self.delay(delaytime)

    def no(self, times):

        for i in range(times):

            self.setServoPulse(self.servoA, self.stA - 60)
            self.delay(200)
            self.setServoPulse(self.servoA, self.stA + 60)
            self.delay(200)

        self.setServoPulse(self.servoA, self.stA)

    def bow(self, delaytime):

        self.bowing()
        self.delay(delaytime)
        self.resetBowing()

    def bye(self, times, dirr):

        self.stand()

        if (dirr == 0):

            self.setServoPulse(self.servoE, self.stE + 80)
            self.setServoPulse(self.servoF, self.stF - 70)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + 80 - j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + j * 10)
                    self.delay(tim)

        if (dirr == 1):
            self.setServoPulse(self.servoB, self.stB - 50)
            self.setServoPulse(self.servoC, self.stC + 80)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServoPulse(self.servoD, self.stD - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServoPulse(self.servoD, self.stD - j * 10)
                    self.delay(tim)

        if (dirr == 2):

            self.setServoPulse(self.servoB, self.stB - 50)
            self.setServoPulse(self.servoC, self.stC + 80)
            self.setServoPulse(self.servoE, self.stE + 50)
            self.setServoPulse(self.servoF, self.stF - 60)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + 80 - j * 10)
                    self.setServoPulse(self.servoD, self.stD - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + j * 10)
                    self.setServoPulse(self.servoD, self.stD - j * 10)
                    self.delay(tim)

    def nadenade(self, times, dirr):

        self.stand()

        if (dirr):

            self.setServoPulse(self.servoC, self.stC + 80)

            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServoPulse(self.servoD, self.stD - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServoPulse(self.servoD, self.stD - j * 10)
                    self.delay(tim)

        if (not dirr):

            self.setServoPulse(self.servoF, self.stF - 60)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + 80 - j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServoPulse(self.servoG, self.stG + j * 10)
                    self.delay(tim)

    def sitDown(self):

        self.sitdown1()
        self.sitdown2()

    def sitUp(self):

        self.situp1()
        self.delay(1000)
        self.situp2()

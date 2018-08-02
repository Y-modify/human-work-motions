# Generated from Actions.ino and ActionFunctions.ino

import math
import time
from threading import Thread


class Motions(object):
    def __init__(self, robot, portmap, stand_positions):
        self.kI = 0.5
        self.robot = robot

        if isinstance(portmap, dict):
            self.portmap = portmap["real"] if self.robot.is_real else portmap["simulator"]
        else:
            self.portmap = portmap

        if isinstance(stand_positions, dict):
            self.stand_positions = stand_positions["real"] if self.robot.is_real else stand_positions["simulator"]
        else:
            self.stand_positions = stand_positions

    def setServoPulse(self, idx, deg):
        if idx is not None:
            self.robot.set_joint_state(idx if self.robot.is_real else idx-8, (deg - 90) / 180 * math.pi)

    def delay(self, ms):
        time.sleep(ms/1000)

    # Initialize Servo Driver
    def ServoInit(self):
        self.delay(100)

    def setServo(self, name, deg):
        self.setServoPulse(self.portmap[name],
                           self.stand_positions[name] + deg)

    # Base Functions
    def stand(self):
        for name, _ in self.stand_positions.items():
            self.setServo(name, 0)

    def bowing(self):
        for i in range(5+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - i * 9)

            self.setServo("K", 0)
            self.setServo("L",  + i * 4)
            self.setServo("M", 0)
            self.setServo("N", 0)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  - i * 4)
            self.setServo("S", 0)
            self.setServo("T", 0)
            self.setServo("U", 0)

            self.delay(40)

    def resetBowing(self):
        for i in range(5+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - 45 + i * 9)

            self.setServo("K", 0)
            self.setServo("L",  + 20 - i * 4)
            self.setServo("M", 0)
            self.setServo("N", 0)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  - 20 + i * 4)
            self.setServo("S", 0)
            self.setServo("T", 0)
            self.setServo("U", 0)

            self.delay(40)

    def crouch(self):
        for i in range(5+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I", 0)

            self.setServo("K", 0)
            self.setServo("L",  - i * 4)
            self.setServo("M",  + i * 8)
            self.setServo("N",  + i * 4)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + i * 4)
            self.setServo("S",  - i * 8)
            self.setServo("T",  - i * 4)
            self.setServo("U", 0)

            self.delay(40)

    def resetCrouch(self):
        for i in range(5+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I", 0)

            self.setServo("K", 0)
            self.setServo("L",  - 20 + i * 4)
            self.setServo("M",  + 40 - i * 8)
            self.setServo("N",  + 20 - i * 4)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + 20 - i * 4)
            self.setServo("S",  - 40 + i * 8)
            self.setServo("T",  - 20 + i * 4)
            self.setServo("U", 0)

            self.delay(40)

    # Walking
    # begins

    def LEFTwalkBeginUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - i * 2.5)
            self.setServo("I",  - i * self.kI)  # モル生えるwww

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + i * 1.25)

            self.delay(frame)

    def LEFTwalkBeginFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI*4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40 + i * 5)
            self.setServo("T",  - 20 + i * 5)
            self.setServo("U",  + 10)
            self.delay(frame)

    def LEFTwalkBeginDown(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 2.5)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20 - i * 0.5)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 20 + i + 0.5)
            self.setServo("S",  - 20 + i * 2.5)
            self.setServo("T",  + i * 3)
            self.setServo("U",  + 10 - i * 1.25)
            self.delay(frame)

    def RIGHTwalkBeginUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + i * 2.5)
            self.setServo("I",  - i * self.kI)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - i * 1.25)

            self.delay(frame)

    def RIGHTwalkBeginFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40 - i * 5)
            self.setServo("N",  + 20 - i * 5)
            self.setServo("O",  - 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 10)
            self.delay(frame)

    def RIGHTwalkBeginDown(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20 - i * 2.5)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 20 - i * 2.5)
            self.setServo("N",  - i * 3)
            self.setServo("O",  - 10 + i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 10 + i * 1.25)
            self.delay(frame)

    def LEFTbackBeginFoward(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI * 4 + i * 2)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 20 - i * 3)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20 - i * 2)
            self.setServo("U",  + 10)
            self.delay(frame)

    def LEFTbackBeginDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  - 4)
            self.setServo("S",  - 40 + i * 4)
            self.setServo("T",  - 36 + i * 2)
            self.setServo("U",  + 10 - i * 2.5)
            self.delay(frame)

    # walking

    def leftEndFoward(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI*4)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + i * 5)
            self.setServo("N",  - 20 + i * 5)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10)

            self.delay(frame)

    def leftEnd(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - self.kI*4 + i * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 24 + i)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 24 - i)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10 - i * 2.5)

            self.delay(frame)

    def leftUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - i * 5)
            self.setServo("I",  - self.kI * 4 - i * 2)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M", 0)
            self.setServo("N",  - 24 + i)
            self.setServo("O",  + i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + i * 2.5)

            self.delay(frame)

    def rightUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + i * 5)
            self.setServo("I",  - self.kI*6 - i * 2)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S", 0)
            self.setServo("T",  + 24 - i)
            self.setServo("U",  - i * 2.5)

            self.delay(frame)

    def leftFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI * 6)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + i * 5)
            self.setServo("N",  - 20 + i * 4.25)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - 40 + i * 5)
            self.setServo("T",  - 20 + i * 5)
            self.setServo("U",  + 10)

            self.delay(frame)

    def rightFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20)
            self.setServo("I",  - self.kI*6)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + 40 - i * 5)
            self.setServo("N",  + 20 - i * 5)
            self.setServo("O",  - 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - i * 5)
            self.setServo("T",  + 20 - i * 4.25)
            self.setServo("U",  - 10)

            self.delay(frame)

    def leftDown(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 2.5)
            self.setServo("I",  - self.kI*6 + i)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + 20 + i * 2.5)
            self.setServo("N",  - 3 + i * 2.875)
            self.setServo("O",  + 10 - i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - 20 + i * 2.5)
            self.setServo("T",  + i * 3)
            self.setServo("U",  + 10 - i * 1.25)

            self.delay(frame)

    def rightDown(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20 - i * 2.5)
            self.setServo("I",  - self.kI*6 + i)

            self.setServo("K", 0)
            self.setServo("L",  - 24)
            self.setServo("M",  + 20 - i * 2.5)
            self.setServo("N",  - i * 3)
            self.setServo("O",  - 10 + i * 1.25)

            self.setServo("Q", 0)
            self.setServo("R",  + 24)
            self.setServo("S",  - 20 - i * 2.5)
            self.setServo("T",  + 3 - i * 2.875)
            self.setServo("U",  - 10 + i * 1.25)

            self.delay(frame)

    # back

    def backLEFTEndFoward(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  + 4 - i * 3)
            self.setServo("M",  + 24 + i * 2)
            self.setServo("N",  + 28 - i)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10)
            self.delay(frame)

    def backLEFTEnd(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - 4 + i)

            self.setServo("K", 0)
            self.setServo("L",  + -20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10 - i * 2.5)
            self.delay(frame)

    def backLEFTUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - i * 5)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  + 4)
            self.setServo("M",  + 24)
            self.setServo("N",  + 28)
            self.setServo("O",  + i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + i * 2.5)
            self.delay(frame)

    def backLEFTFoward(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  + 4 - i * 3)
            self.setServo("M",  + 24 + i * 2)
            self.setServo("N",  + 28 - i)
            self.setServo("O",  + 10)

            self.setServo("Q", 0)
            self.setServo("R",  + 20 - i * 3)
            self.setServo("S",  - 40 + i * 2)
            self.setServo("T",  - 20 - i)
            self.setServo("U",  + 10)
            self.delay(frame)

    def backLEFTDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  - 4)
            self.setServo("S",  - 24)
            self.setServo("T",  - 28)
            self.setServo("U",  + 10 - i * 2.5)
            self.delay(frame)

    def backRIGHTUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + i * 5)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  - 4)
            self.setServo("S",  - 24)
            self.setServo("T",  - 28)
            self.setServo("U",  - i * 2.5)
            self.delay(frame)

    def backRIGHTFoward(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20 + i * 3)
            self.setServo("M",  + 40 - i * 2)
            self.setServo("N",  + 20 + i)
            self.setServo("O",  - 10)

            self.setServo("Q", 0)
            self.setServo("R",  - 4 + i * 3)
            self.setServo("S",  - 24 - i * 2)
            self.setServo("T",  - 28 + i)
            self.setServo("U",  - 10)
            self.delay(frame)

    def backRIGHTDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20 - i * 5)
            self.setServo("I",  - 4)

            self.setServo("K", 0)
            self.setServo("L",  + 4)
            self.setServo("M",  + 24)
            self.setServo("N",  + 28)
            self.setServo("O",  - 10 + i * 2.5)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 10 + i * 2.5)
            self.delay(frame)

    def backRIGHTToCrouch(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  + 4 - i * 3)
            self.setServo("M",  + 24 + i * 2)
            self.setServo("N",  + 28 - i)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U", 0)
            self.delay(frame)

    # turn

    def turnRightLEFTToCrouch(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - i * 5)
            self.setServo("T",  + 24 - i * 5.5)
            self.setServo("U", 0)
            self.delay(frame)

    def turnLeftRIGHTToCrouch(self, frame):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - self.kI * 4)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + i * 5)
            self.setServo("N",  - 24 + i * 5.5)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U", 0)
            self.delay(frame)

    # crab

    def RIGHTrightCrabFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - i * 2)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - 10)

            self.setServo("Q",  + i * 2)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 10)

            self.delay(frame)

    def RIGHTrightCrabDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20 - i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - 10 + i * 4)

            self.setServo("Q",  + 8)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 10 + i)

            self.delay(frame)

    def RIGHTleftCrabUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 6 - i * 2.5)

            self.setServo("Q",  + 8)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 6 + i * 2)

            self.delay(frame)

    def RIGHTleftCrabFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8 + i * 2)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - 4)

            self.setServo("Q",  + 8 - i * 2)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 4)

            self.delay(frame)

    def RIGHTleftCrabDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - 4 + i)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 4 - i)

            self.delay(frame)

    def LEFTleftCrabFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - i * 2)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10)

            self.setServo("Q",  + i * 2)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10)

            self.delay(frame)

    def LEFTleftCrabDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  - 20 + i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 10 - i * 4)

            self.setServo("Q",  + 8)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 10 - i)

            self.delay(frame)

    def LEFTrightCrabUp(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  - 6 + i * 2.5)

            self.setServo("Q",  + 8)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  + 6 - i * 2)

            self.delay(frame)

    def LEFTrightCrabFoward(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K",  - 8 + i * 2)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 4)

            self.setServo("Q",  + 8 - i * 2)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 4)

            self.delay(frame)

    def LEFTrightCrabDown(self, frame):
        for i in range(4+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H",  + 20 - i * 5)
            self.setServo("I",  - self.kI * 4 + 5)

            self.setServo("K", 0)
            self.setServo("L",  - 20)
            self.setServo("M",  + 40)
            self.setServo("N",  + 20)
            self.setServo("O",  + 4 - i)

            self.setServo("Q", 0)
            self.setServo("R",  + 20)
            self.setServo("S",  - 40)
            self.setServo("T",  - 20)
            self.setServo("U",  - 4 + i)

            self.delay(frame)

    # dogeza

    def seiza1(self):
        for i in range(10+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I", 0)

            self.setServo("K", 0)
            self.setServo("L",  - i * 3)
            self.setServo("M",  + i * 7)
            self.setServo("N",  + i * 4)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + i * 3)
            self.setServo("S",  - i * 7)
            self.setServo("T",  - i * 4)
            self.setServo("U", 0)

            self.delay(40)

    # sit

    def sitdown1(self):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - i * 4.5)

            self.setServo("K", 0)
            self.setServo("L",  + i * 2)
            self.setServo("M", 0)
            self.setServo("N", 0)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  - i * 2)
            self.setServo("S", 0)
            self.setServo("T", 0)
            self.setServo("U", 0)

            self.delay(50)

    def sitdown2(self):
        for i in range(16+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - 36 - i * 2.25)

            self.setServo("K", 0)
            self.setServo("L",  + 16 + i * 0.5)
            self.setServo("M",  + i)
            self.setServo("N",  - i * 1.5)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  - 16 - i * 0.5)
            self.setServo("S",  - i)
            self.setServo("T",  + i * 1.5)
            self.setServo("U", 0)

            self.delay(50)

    def situp1(self):
        for i in range(8+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - 72)

            self.setServo("K", 0)
            self.setServo("L",  + 24 - i * 6)
            self.setServo("M",  + 16 - i)
            self.setServo("N",  - 24)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  - 24 + i * 6)
            self.setServo("S",  - 16 + i)
            self.setServo("T",  + 24)
            self.setServo("U", 0)

            self.delay(60)

    def situp2(self):
        for i in range(16+1):
            self.setServo("A", 0)

            self.setServo("B", 0)
            self.setServo("C", 0)
            self.setServo("D", 0)

            self.setServo("E", 0)
            self.setServo("F", 0)
            self.setServo("G", 0)

            self.setServo("H", 0)
            self.setServo("I",  - 72 + i * 4.5)

            self.setServo("K", 0)
            self.setServo("L",  - 24 + i * 1.5)
            self.setServo("M",  + 8 - i * 0.5)
            self.setServo("N",  - 24 + i * 1.5)
            self.setServo("O", 0)

            self.setServo("Q", 0)
            self.setServo("R",  + 24 - i * 1.5)
            self.setServo("S",  - 8 + i * 0.5)
            self.setServo("T",  + 24 - i * 1.5)
            self.setServo("U", 0)

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

            self.setServo("A",  - 60)
            self.delay(200)
            self.setServo("A",  + 60)
            self.delay(200)

        self.setServo("A", 0)

    def bow(self, delaytime):

        self.bowing()
        self.delay(delaytime)
        self.resetBowing()

    def bye(self, times, dirr):

        self.stand()

        if (dirr == 0):

            self.setServo("E",  + 80)
            self.setServo("F",  - 70)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServo("G",  + 80 - j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServo("G",  + j * 10)
                    self.delay(tim)

        if (dirr == 1):
            self.setServo("B",  - 50)
            self.setServo("C",  + 80)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServo("D",  - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServo("D",  - j * 10)
                    self.delay(tim)

        if (dirr == 2):

            self.setServo("B",  - 50)
            self.setServo("C",  + 80)
            self.setServo("E",  + 50)
            self.setServo("F",  - 60)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServo("G",  + 80 - j * 10)
                    self.setServo("D",  - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServo("G",  + j * 10)
                    self.setServo("D",  - j * 10)
                    self.delay(tim)

    def nadenade(self, times, dirr):

        self.stand()

        if (dirr):

            self.setServo("C",  + 80)

            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServo("D",  - 80 + j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServo("D",  - j * 10)
                    self.delay(tim)

        if (not dirr):

            self.setServo("F",  - 60)
            for i in range(times):

                tim = 20
                for j in range(8):

                    self.setServo("G",  + 80 - j * 10)
                    self.delay(tim)

                for j in range(8):

                    self.setServo("G",  + j * 10)
                    self.delay(tim)

    def sitDown(self):

        self.sitdown1()
        self.sitdown2()

    def sitUp(self):

        self.situp1()
        self.delay(1000)
        self.situp2()
